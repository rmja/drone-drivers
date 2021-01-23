pub trait Cc1200Chip<A> {
    fn select(&mut self);
    fn deselect(&mut self);
}
