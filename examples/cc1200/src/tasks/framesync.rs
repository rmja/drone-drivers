use alloc::collections::VecDeque;
use drone_cc1200_drv::controllers::infinite::RxChunk;
use drone_core::sync::spsc::ring::{Receiver, Sender};
use drone_framesync::{SyncWindow, FrameBuffer, detectors::{cortexm4::{sync16_tol1, sync32_tol2}, Detector}};
use drone_mbus::{modec, WMBusPacket};
use drone_stm32f4_hal::{prelude::*, gpio::{GpioPin, GpioPinMap}};
use futures::StreamExt;

use crate::consts::Tim2Tick;

use super::{Packet, rf::RfError};


pub async fn handler<Pin: GpioPinMap>(mut rf_stream: Receiver<Result<RxChunk<Tim2Tick>, RfError>, ()>, mut packet_stream: Sender<Packet, ()>, busy: GpioPin<Pin, OutputMode, PushPullType, NoPull>) {
    let mut window = SyncWindow::new(sync32_tol2::<0x543D543D>());
    let mut ongoing_frames: Vec<FrameBuffer> = vec![];

    // Wait for chunks of bytes to be received from the transceiver.
    while let Some(chunk) = rf_stream.next().await {
        busy.set();
        let chunk = chunk.unwrap();
        if let Ok(chunk) = chunk {
            // This chunk was received correctly without any interruptions in the stream.

            // Add the received bytes into all ongoing, concurrent receiptions.
            for frame in ongoing_frames.iter_mut() {
                frame.receive_buffer.extend_from_slice(&chunk.bytes);
            }

            // Look for syncword in the chunk
            window.extend(&chunk.bytes);
            while let Some((shifts, remainder)) = window.detect().next() {
                ongoing_frames.push(FrameBuffer {
                    receive_buffer: remainder,
                    shifts,
                    frame_len: None, // Not yet determined
                });
            }
        }
        else {
            // Some kind of error occured in the chunk stream.
            // Reset the sync window and clear all ongoing receiptions.
            window = SyncWindow::new(sync32_tol2::<0x543D543D>());
            ongoing_frames.clear();
        }
            
        let drained: Vec<FrameBuffer> = ongoing_frames.
            drain_filter(|frame| {
                if frame.frame_len.is_none() && frame.aligned_len() > 4 {
                    // We have at least the syncword and the length
                    let length_field = frame.get_aligned_part(4..5)[0];
                    frame.frame_len = Some(4 + 1 + usize::from(length_field));
                }

                if frame.is_received() {
                    let mut frame_bytes = frame.get_aligned();
                    frame_bytes.drain(0..4); // Remove the syncword

                    let packet = WMBusPacket::parse_ffb(&frame_bytes);
                    if let Ok(packet) = packet {
                        packet_stream.send(Packet::WMBus(packet)).map_err(|e| "err").unwrap();
                    }
                    else {
                        packet_stream.send(Packet::Invalid(frame_bytes)).map_err(|e| "err").unwrap();
                    }

                    // Remove the frame from the list of ongoing frames
                    true
                }
                else {
                    // Keep the frame - it is not fully received yet
                    false
                }
            }).collect();

        busy.clear();
    }
}