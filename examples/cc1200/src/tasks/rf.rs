use alloc::sync::Arc;
use drone_cc1200_drv::{Cc1200Chip, Cc1200Drv, Strobe, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, configs::CC1200_WMBUS_MODECMTO_FULL, controllers::{DebugController, InfiniteController}};
use drone_core::sync::Mutex;
use drone_stm32f4_hal::{IntToken, dma::DmaChMap, exti::ExtiMap, gpio::{PinAf, GpioPinMap}, spi::{SpiMap, SpiMasterDrv}};
use drone_time::{Alarm, Tick};
use futures::prelude::*;

use crate::{adapters, consts::Tim2Tick};

pub async fn handler<
    ResetPin: GpioPinMap,
    MisoPin: GpioPinMap,
    MisoAf: PinAf,
    MisoExti: ExtiMap,
    MisoExtiInt: IntToken,
    Al: Alarm<Tim2Tick>,
    // Spi: Cc1200Spi<A> + Send,
    Spi: SpiMap,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,

    Chip: Cc1200Chip<A> + Send + 'static,
    A: Send + 'static,
>(port: adapters::cc1200::Port<ResetPin, MisoPin, MisoAf, MisoExti, MisoExtiInt>, alarm: Arc<Al>, spi: Arc<Mutex<SpiMasterDrv<Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>>>, mut chip: Chip) {
    loop
    {
        let mut spi = spi.lock().await;
        // Code cannot compile if this line is included
        // spi.write(&[0x00]).await;



        // let drv = Cc1200Drv::init(port, alarm);
        // // let mut debug = DebugController::setup(drv, spi, chip, &CC1200_WMBUS_MODECMTO_FULL).await.unwrap();
        // drv.strobe(&mut *spi, &mut chip, Strobe::SIDLE).await;


        // let mut debug = DebugController::setup(drv, spi, chip, &CC1200_WMBUS_MODECMTO_FULL).await.unwrap();

        
        // let mut ctrl = ctrl.try_lock().unwrap();
        // loop {
        //     let mut chunk_stream = ctrl.receive_stream(4).await;
        //     while let Some(chunk) = chunk_stream.next().await {

        //     }
        // }
    }
}
