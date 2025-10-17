/*!
Morphology rules for synaptogenesis.

Implements various connectivity patterns:
- Projector: Map source neurons to destination maintaining topology
- Block connections: Scaled block mapping
- Expander: Scale coordinates from source to destination
- Reducer: Binary encoding/decoding
*/

mod projector;
mod block_connection;
mod expander;
mod reducer;

pub use projector::{syn_projector, syn_projector_batch, ProjectorParams};
pub use block_connection::syn_block_connection;
pub use expander::{syn_expander, syn_expander_batch};
pub use reducer::syn_reducer_x;

