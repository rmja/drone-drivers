use async_trait::async_trait;

#[async_trait]
pub trait Cc1200Port {
    fn set_reset(&mut self);
    fn clear_reset(&mut self);
    async fn miso_wait_low(&mut self);
}
