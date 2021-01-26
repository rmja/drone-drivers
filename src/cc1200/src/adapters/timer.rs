use alloc::{boxed::Box, sync::Arc};
use core::pin::Pin;
use futures::Stream;

pub trait Cc1200Timer<A> {
    /// Get a pin handle to be used to get the current capture pin state.
    fn pin(&self) -> Arc<dyn Cc1200TimerPin<A>>;

    /// Get a stream of timer capture values.
    fn capture_overwriting_stream<'a>(
        &'a mut self,
        capacity: usize,
    ) -> Pin<Box<dyn Stream<Item = u32> + 'a>>;
}

pub trait Cc1200TimerPin<A>: Send {
    /// Get the current pin state.
    fn get(&self) -> bool;
}
