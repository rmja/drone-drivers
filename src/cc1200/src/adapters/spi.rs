use async_trait::async_trait;

#[async_trait]
pub trait Cc1200Spi<A> {
    async fn xfer(&mut self, tx: &[u8], rx: &mut [u8]);
}
