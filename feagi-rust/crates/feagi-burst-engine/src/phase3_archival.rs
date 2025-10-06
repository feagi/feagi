/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Phase 3: Archival
//!
//! Records firing neurons to the Fire Ledger for historical tracking.

use feagi_types::*;

/// Phase 3: Archival
///
/// Records the current burst's firing to the Fire Ledger
pub fn phase3_archival(
    fire_queue: &FireQueue,
    fire_ledger: &mut FireLedger,
    burst: u64,
) -> Result<()> {
    let neuron_ids = fire_queue.get_all_neuron_ids();
    fire_ledger.record_burst(burst, neuron_ids);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_archival() {
        let mut fire_queue = FireQueue::new();
        fire_queue.add_neuron(FiringNeuron {
            neuron_id: NeuronId(1),
            membrane_potential: 1.5,
            cortical_area: CorticalAreaId(1),
            x: 0, y: 0, z: 0,
        });
        fire_queue.add_neuron(FiringNeuron {
            neuron_id: NeuronId(2),
            membrane_potential: 2.0,
            cortical_area: CorticalAreaId(1),
            x: 1, y: 0, z: 0,
        });
        
        let mut ledger = FireLedger::new(10);
        
        phase3_archival(&fire_queue, &mut ledger, 100).unwrap();
        
        let history = ledger.get_burst(100).unwrap();
        assert_eq!(history.burst, 100);
        assert_eq!(history.neurons.len(), 2);
    }
}
