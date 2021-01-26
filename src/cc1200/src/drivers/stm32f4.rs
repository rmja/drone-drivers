use crate::{Cc1200Chip, Cc1200Spi, Cc1200Timer, Cc1200TimerPin};
use alloc::sync::Arc;
use async_trait::async_trait;
use drone_stm32f4_hal::{IntToken, dma::DmaChMap, gpio::{AlternateMode, GpioPin, GpioPinMap, PinAf}, spi::{chipctrl::SpiChip, SpiMap, SpiMasterDrv}, tim::{CaptureStop, CaptureStream, GeneralTimCh, GeneralTimChDrv, GeneralTimMap, InputCaptureMode, InputSelection, TimerCaptureCh, TimerPinCaptureCh}};
use futures::Stream;

pub struct Adapter;

impl<Pin: GpioPinMap, PinType, PinPull> Cc1200Chip<Adapter> for SpiChip<Pin, PinType, PinPull> {
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
    Cc1200Spi<Adapter> for SpiMasterDrv<'_, Spi, DmaRx, DmaRxInt, DmaTx, DmaTxInt>
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
    Type: Send + Sync + 'static,
    Pull: Send + Sync + 'static,
    Sel: InputSelection,
> Cc1200Timer<Adapter> for GeneralTimChDrv<Tim, Int, Ch, InputCaptureMode<Pin, Af, Type, Pull, Sel>> {
    fn pin(&self) -> Arc<dyn Cc1200TimerPin<Adapter>> {
        TimerPinCaptureCh::pin(self)
    }

    #[inline]
    fn capture_overwriting_stream<'a>(&'a mut self, capacity: usize) -> core::pin::Pin<Box<dyn Stream<Item = u32> + 'a>> {
        Box::pin(TimerCaptureCh::overwriting_stream(self, capacity))
    }
}

impl<Pin: GpioPinMap, Af: PinAf, Type: Send, Pull: Send> Cc1200TimerPin<Adapter> for GpioPin<Pin, AlternateMode<Af>, Type, Pull> {
    fn get(&self) -> bool {
        self.get()
    }
}