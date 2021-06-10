use drone_core::sync::spsc::ring::Receiver;
use drone_mbus::WMBusPacket;
use futures::StreamExt;

pub async fn handler(mut packet_stream: Receiver<WMBusPacket, ()>, serial: u32) {
    // while let packet = serial_stream.next().await {
    //     let packet = packet.unwrap().unwrap();

    // }
}