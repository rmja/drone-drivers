use async_trait::async_trait;

#[async_trait]
pub trait At250x0Timer<A> {
    async fn sleep_ns(&mut self, duration: u32);
    async fn sleep_us(&mut self, duration: u32);
}
