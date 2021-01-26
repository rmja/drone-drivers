use drone_time::{Tick, TimeSpan};

pub trait Cc1200Uptime<T: Tick, A> {
    fn upstamp(&self, capture: u32) -> TimeSpan<T>;
}
