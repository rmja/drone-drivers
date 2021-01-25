use core::{pin::Pin, task::{Context, Poll}};

use futures::Stream;
use alloc::boxed::Box;

pub trait Cc1200Timer<A> {
    // type Ctrl: Cc1200CaptureControl;

    fn clear_pending_capture(&mut self);
    fn capture_overwriting_stream<'a>(&'a mut self, capacity: usize) -> Pin<Box<dyn Stream<Item = u32> + 'a>>;
}

// pub trait Cc1200CaptureControl: Send {
//     /// Get the current value of the capture pin.
//     fn get(&self) -> bool;

//     /// Stop the capture stream.
//     fn stop(&mut self);
// }

// pub struct Cc1200CaptureStream<'a, Ctrl: Cc1200CaptureControl> {
//     ctrl: &'a mut Ctrl,
//     stream: Pin<Box<dyn Stream<Item = u32> + Send + 'a>>,
// }

// impl<'a, Ctrl: Cc1200CaptureControl> Cc1200CaptureStream<'a, Ctrl> {
//     pub fn new(stop: &'a mut Ctrl, stream: Pin<Box<dyn Stream<Item = u32> + Send + 'a>>) -> Self {
//         Self { ctrl: stop, stream }
//     }

//     pub fn get(&self) -> bool {
//         self.ctrl.get()
//     }

//     /// Stop the capture stream.
//     #[inline]
//     pub fn stop(mut self: Pin<&mut Self>) {
//         self.ctrl.stop();
//     }
// }

// impl<Ctrl: Cc1200CaptureControl> Stream for Cc1200CaptureStream<'_, Ctrl> {
//     type Item = u32;

//     #[inline]
//     fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
//         self.stream.as_mut().poll_next(cx)
//     }
// }

// impl<Ctrl: Cc1200CaptureControl> Drop for Cc1200CaptureStream<'_, Ctrl> {
//     #[inline]
//     fn drop(&mut self) {
//         self.ctrl.stop();
//     }
// }