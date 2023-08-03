pub use crate::command::Command;
use crate::config::Config;
/// native rust function
pub fn sum(a: usize, b: usize) -> String {
    println!("call in rust {:?} {:?}", a, b);
    (a + b).to_string()
}
