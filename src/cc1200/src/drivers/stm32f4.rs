use crate::{Cc1200Chip, Cc1200Spi, Cc1200Timer, Cc1200TimerPin};
use alloc::sync::Arc;
use async_trait::async_trait;
use drone_stm32f4_hal::{IntToken, dma::DmaChMap, gpio::{GpioPin, GpioPinMap, prelude::*}, spi::{chipctrl::SpiChip, SpiMap, SpiMasterDrv}, tim::{GeneralTimCh, GeneralTimChDrv, GeneralTimMap, InputCaptureMode, InputSelection, TimerCaptureCh, TimerCapturePolarity, TimerPinCaptureCh}};
use futures::Stream;

pub struct Adapter;

impl<Pin: GpioPinMap, PinType: PinTypeMap> Cc1200Chip<Adapter> for SpiChip<Pin, PinType, PullUp> {
    #[inline]
    fn select(&mut self) {
        self.select();
    }

    #[inline]
    fn deselect(&mut self) {
        self.deselect();
    }
}

#[async_trait]
impl<Spi: SpiMap, DmaRx: DmaChMap, DmaRxInt: IntToken, DmaTx: DmaChMap, DmaTxInt: IntToken>
    Cc1200Spi<Adapter> for SpiMasterDrv<Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>
{
    #[inline]
    async fn read(&mut self, rx: &mut [u8]) {
        self.read(rx).await;
    }

    #[inline]
    async fn write(&mut self, tx: &[u8]) {
        self.write(tx).await;
    }

    #[inline]
    async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]) {
        self.xfer(tx, rx).await;
    }
}

impl<
        Tim: GeneralTimMap,
        Int: IntToken,
        Ch: GeneralTimCh<Tim>,
        Pin: GpioPinMap,
        Af: PinAf,
        Type: PinTypeMap,
        Pull: PinPullMap,
        Sel: InputSelection,
    > Cc1200Timer<Adapter>
    for GeneralTimChDrv<Tim, Int, Ch, InputCaptureMode<Pin, Af, Type, Pull, Sel>>
{
    fn pin(&self) -> Arc<dyn Cc1200TimerPin<Adapter>> {
        TimerPinCaptureCh::pin(self)
    }

    #[inline]
    fn rising_edge_capture_overwriting_stream<'a>(
        &'a mut self,
        capacity: usize,
    ) -> core::pin::Pin<Box<dyn Stream<Item = u32> + 'a>> {
        Box::pin(TimerCaptureCh::overwriting_stream(self, capacity, TimerCapturePolarity::RisingEdge))
    }

    #[inline]
    fn falling_edge_capture_overwriting_stream<'a>(
        &'a mut self,
        capacity: usize,
    ) -> core::pin::Pin<Box<dyn Stream<Item = u32> + 'a>> {
        Box::pin(TimerCaptureCh::overwriting_stream(self, capacity, TimerCapturePolarity::FallingEdge))
    }
}

impl<Pin: GpioPinMap, Af: PinAf, Type: PinTypeMap, Pull: PinPullMap> Cc1200TimerPin<Adapter>
    for GpioPin<Pin, AlternateMode<Af>, Type, Pull>
{
    fn get(&self) -> bool {
        self.get()
    }
}
