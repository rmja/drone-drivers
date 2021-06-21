use alloc::sync::Arc;
use drone_cc1200_drv::{Cc1200Chip, Cc1200Drv, Cc1200Gpio, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, RxFifoOverflowError, configs::CC1200_WMBUS_MODECMTO_FULL_INFINITY, controllers::infinite::{InfiniteController, RxChunk}};
use drone_core::sync::{Mutex, spsc::ring::{SendErrorKind, Sender}};
use drone_time::Alarm;
use futures::prelude::*;

use crate::consts::Tim2Tick;

#[derive(Debug)]
pub struct RxOfflineError;

pub async fn handler<
    Port: Cc1200Port,
    Al: Alarm<Tim2Tick>,
    Spi: Cc1200Spi<A> + Send,
    Chip: Cc1200Chip<A> + Send,
    Timer: Cc1200Timer<A> + Send,
    Upt: Cc1200Uptime<Tim2Tick, A> + Send,
    A: Send,
>(port: Port, alarm: Arc<Al>, spi: Arc<Mutex<Spi>>, mut chip: Chip, timer: Timer, uptime: Arc<Upt>, mut sender: Sender<Result<RxChunk<Tim2Tick>, RxFifoOverflowError>, RxFifoOverflowError>) {

    let drv = Cc1200Drv::init(port, alarm);

    // Issue hardware reset sequence
    drv.hw_reset(&mut chip).await.unwrap();

    let mut ctrl = InfiniteController::setup(drv, spi, chip, timer, uptime, &CC1200_WMBUS_MODECMTO_FULL_INFINITY, Cc1200Gpio::Gpio0).await.unwrap();
    
    loop {
        let mut chunk_stream = ctrl.receive(4).await;
        while let Some(chunk) = chunk_stream.next().await {
            let send_result = sender.send(chunk);

            match send_result {
                Err(error) if error.kind == SendErrorKind::Overflow => {
                    sender.send_overwrite(Err(RxFifoOverflowError)).unwrap();
                },
                _ => send_result.unwrap(),
            }
        }
    }
}
