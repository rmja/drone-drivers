use alloc::collections::VecDeque;
use drone_cc1200_drv::{RxFifoOverflowError, controllers::infinite::RxChunk};
use drone_core::sync::spsc::ring::{Receiver, Sender};
use drone_framesync::{SyncWindow, FrameBuffer, detectors::{cortexm4::{sync16_tol1, sync32_tol2}, Detector}};
use drone_mbus::{modec, WMBusPacket};
use futures::StreamExt;

use crate::consts::Tim2Tick;

pub async fn handler(mut rf_stream: Receiver<Result<RxChunk<Tim2Tick>, RxFifoOverflowError>, RxFifoOverflowError>, mut packet_stream: Sender<WMBusPacket, ()>) {
    let modec_detector = sync32_tol2::<0x543D543D>();
    let mut window = SyncWindow::new(modec_detector);

    let mut ongoing_frames: Vec<FrameBuffer> = vec![];

    // Wait for chunks of bytes to be received from the transceiver.
    while let Some(chunk) = rf_stream.next().await {
        let chunk = chunk.unwrap().unwrap();

        // // Add the received bytes into all ongoing, concurrent receiptions.
        // for frame in ongoing_frames.iter_mut() {
        //     frame.receive_buffer.extend_from_slice(&chunk.bytes);
        // }

        // // Look for syncword in the chunk
        // window.extend(&chunk.bytes);
        // while let Some((shifts, remainder)) = window.detect().next() {
        //     ongoing_frames.push(FrameBuffer {
        //         receive_buffer: remainder,
        //         shifts,
        //         frame_len: None, // Not yet determined
        //     });
        // }
        
        // ongoing_frames.drain_filter(|frame| {
        //     if frame.frame_len.is_none() && frame.receive_buffer.len() > 4 {
        //         // We have at least the syncword and the length
        //         let length_field = frame.get_aligned_part(4..5)[0];
        //         frame.frame_len = Some(4 + 1 + usize::from(length_field));
        //     }

        //     if frame.is_received() {
        //         let aligned_frame_bytes = frame.get_aligned();
        //         let packet = WMBusPacket::parse_ffb(&aligned_frame_bytes);
        //         if let Ok(packet) = packet {
        //             packet_stream.send(packet);
        //         }

        //         // Remove the frame from the list of ongoing frames
        //         true
        //     }
        //     else {
        //         // Keep the frame - it is not fully received yet
        //         false
        //     }
        // });
    }
}