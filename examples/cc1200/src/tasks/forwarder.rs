use drone_core::sync::spsc::ring::Receiver;
use drone_stm32f4_hal::{IntToken, dma::DmaChMap, gpio::{GpioPin, GpioPinMap}, prelude::*, uart::{UartMap, UartTxDrv}};
use futures::StreamExt;

use super::Packet;

pub async fn handler<
    Uart: UartMap,
    UartInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    Pin: GpioPinMap,
>(mut packet_stream: Receiver<Packet, ()>, mut serial: UartTxDrv<Uart, UartInt, DmaTx, DmaTxInt>, mut busy: GpioPin<Pin, OutputMode, PushPullType, NoPull>) {
    let mut tx = serial.start();

    // Listen for packets from the frame synchronizer thread
    while let Some(packet) = packet_stream.next().await {
        busy.set();
        let packet = packet.unwrap();

        match packet {
            // Write packet to the uart
            Packet::WMBus(packet) => {
                tx.write(b"WMBus:").await;
                tx.write(&packet.application_layer.data).await;
            },
            Packet::Invalid(frame_bytes) => tx.write(&frame_bytes).await,
        };
        busy.clear();
    }
}