use alloc::sync::Arc;
use drone_cc1200_drv::{Cc1200Chip, Cc1200Drv, Strobe, Cc1200Port, Cc1200Spi, Cc1200Timer, Cc1200Uptime, configs::CC1200_WMBUS_MODECMTO_FULL, controllers::{DebugController, InfiniteController}};
use drone_core::sync::Mutex;
use drone_stm32f4_hal::{IntToken, dma::DmaChMap, exti::ExtiMap, gpio::{GpioPinMap, PinAf, PullUp, PushPullType}, spi::{SpiMap, SpiMasterDrv, chipctrl::SpiChip}};
use drone_time::{Alarm, Tick};
use futures::prelude::*;

use crate::{adapters, consts::{self, Tim2Tick}};

pub async fn handler<
ResetPin: GpioPinMap,
// MisoPin: GpioPinMap,
// MisoAf: PinAf,
// MisoExti: ExtiMap,
// MisoExtiInt: IntToken,

Spi: SpiMap,
DmaRx: DmaChMap,
DmaRxInt: IntToken,
DmaTx: DmaChMap,
DmaTxInt: IntToken,

CsPin: GpioPinMap,

    // Port: Cc1200Port + Send + 'static,
    Al: Alarm<Tim2Tick> + Send + Sync + 'static,
    // Spi: Cc1200Spi<drone_cc1200_drv::drivers::stm32f4::Adapter> + Send + 'static,
    // Chip: Cc1200Chip<drone_cc1200_drv::drivers::stm32f4::Adapter> + Send + 'static,
>(port: adapters::cc1200::Port<ResetPin/*, MisoPin, MisoAf, MisoExti, MisoExtiInt*/>, alarm: Arc<Al>, spi: Arc<Mutex<SpiMasterDrv<Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>>>, mut chip: SpiChip<CsPin, PushPullType, PullUp>) {
    
        // let mut spi = spi.lock().await;

        // spi.write(&[1,2,3]).await;
        let drv: Cc1200Drv<adapters::cc1200::Port<ResetPin/*, MisoPin, MisoAf, MisoExti, MisoExtiInt*/>, Al, Tim2Tick, drone_cc1200_drv::drivers::stm32f4::Adapter> = Cc1200Drv::init(port, alarm);
        // // let mut debug = DebugController::setup(drv, spi, chip, &CC1200_WMBUS_MODECMTO_FULL).await.unwrap();
        // drv.strobe(&mut *spi, &mut chip, Strobe::SIDLE).await;

        drv.testing().await;


        // let mut debug = DebugController::setup(drv, spi, chip, &CC1200_WMBUS_MODECMTO_FULL).await.unwrap();

        
        // let mut ctrl = ctrl.try_lock().unwrap();
        // loop {
        //     let mut chunk_stream = ctrl.receive_stream(4).await;
        //     while let Some(chunk) = chunk_stream.next().await {

        //     }
        // }
    // loop
    // {
    // }
}
