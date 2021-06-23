//! The tasks.

pub mod root;
pub mod rf;
pub mod framesync;
pub mod forwarder;

use drone_mbus::WMBusPacket;

pub use self::root::handler as root;
pub use self::forwarder::handler as forwarder;
pub use self::rf::handler as rf;
pub use self::framesync::handler as framesync;

pub enum Packet {
    WMBus(WMBusPacket),
    Invalid(Vec<u8>),
}