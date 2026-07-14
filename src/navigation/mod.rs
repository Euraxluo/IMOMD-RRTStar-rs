//! Event-driven, algorithm-pluggable realtime navigation runtime.

pub mod events;
pub mod gate;
pub mod imomd_plugin;
pub mod plugin;
pub mod session;
pub mod shortest;
pub mod tour;

pub use events::{DomainEvent, PlanUpdate, UpdateReason};
pub use imomd_plugin::ImomdPlugin;
pub use plugin::PlannerPlugin;
pub use session::NavigationSession;
pub use tour::{EXACT_MAX_OBJECTIVES, solve_exact_tour, solve_greedy_tour};
