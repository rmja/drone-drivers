use alloc::collections::VecDeque;
use drone_cc1200_drv::{RxFifoOverflowError, controllers::infinite::RxChunk};
use drone_core::sync::spsc::ring::Receiver;
use drone_framesync::detectors::{cortexm4::sync32_tol2, Detector};
use futures::StreamExt;
use bitvec::prelude::*;

use crate::consts::Tim2Tick;

pub async fn handler(mut receiver: Receiver<Result<RxChunk<Tim2Tick>, RxFifoOverflowError>, RxFifoOverflowError>) {
    let detector = sync32_tol2::<0xFFFFFFFF>();

    // let bufffer = bitvec![Msb0, u8; 0; 0];

    let mut buffer = VecDeque::new();

    while let Some(chunk) = receiver.next().await {
        let chunk = chunk.unwrap().unwrap();

        buffer.extend(chunk.bytes);

        let (first, second) = buffer.as_slices();
        
        if second.is_empty() {
            detector.position(first);
        }


        detector.position(&chunk.bytes);
    }
}