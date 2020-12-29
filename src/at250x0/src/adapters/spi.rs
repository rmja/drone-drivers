use async_trait::async_trait;

#[async_trait]
pub trait At250x0Spi<A> {
    async fn read(&mut self, rx: &mut [u8]);
    async fn write(&mut self, tx: &[u8]);
    async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]);
}
