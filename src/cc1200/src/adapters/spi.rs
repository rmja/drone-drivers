use async_trait::async_trait;

#[async_trait]
pub trait Cc1200Spi<A> {
    async fn write(&mut self, tx: &[u8]);
    async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]);
}
