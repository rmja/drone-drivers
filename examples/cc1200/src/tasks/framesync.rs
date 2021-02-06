use alloc::collections::VecDeque;
use drone_cc1200_drv::{RxFifoOverflowError, controllers::infinite::RxChunk};
use drone_core::sync::spsc::ring::Receiver;
use drone_framesync::{SyncWindow, detectors::{cortexm4::sync32_tol2, Detector}};
use futures::StreamExt;

use crate::consts::Tim2Tick;

pub async fn handler(mut receiver: Receiver<Result<RxChunk<Tim2Tick>, RxFifoOverflowError>, RxFifoOverflowError>) {
    let detector = sync32_tol2::<0xFFFFFFFF>();
    let mut sync = SyncWindow::new(detector);

    while let Some(chunk) = receiver.next().await {
        let chunk = chunk.unwrap().unwrap();

        sync.extend(&chunk.bytes);

        
    }
}