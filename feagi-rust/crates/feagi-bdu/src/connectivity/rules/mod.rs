/*!
Morphology rules for synaptogenesis.

Implements various connectivity patterns:
- Projector: Map source neurons to destination maintaining topology
- Neighbor finder: Local connectivity
- Block connections: Scaled block mapping
- Patterns: Custom connection patterns
*/

mod projector;

pub use projector::{syn_projector, syn_projector_batch, ProjectorParams};

