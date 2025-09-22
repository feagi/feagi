#!/usr/bin/env python3
"""
Fix syntax errors in NPU tests caused by automated fixing.
"""

import re

def fix_syntax_errors(file_path):
    """Fix syntax errors in a specific file."""
    
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Fix cases where len(neuron_ids) is used but neuron_ids is not available
    # Replace with actual array lengths based on the context
    
    # Fix pattern: neuron_ids=[1, 2, 3] ... decay_rates=[0.95] * len(neuron_ids)
    def fix_len_neuron_ids(match):
        full_match = match.group(0)
        # Count actual neuron_ids in the neuron_ids parameter
        neuron_ids_match = re.search(r'neuron_ids=\[([^\]]+)\]', full_match)
        if neuron_ids_match:
            neuron_ids_str = neuron_ids_match.group(1)
            # Count the number of elements
            if ',' in neuron_ids_str:
                count = len([x.strip() for x in neuron_ids_str.split(',') if x.strip()])
            else:
                count = 1
            
            # Replace len(neuron_ids) with the actual count
            fixed_match = re.sub(r'\[0\.95\] \* len\(neuron_ids\)', f'[0.95] * {count}', full_match)
            fixed_match = re.sub(r'\[1\] \* len\(neuron_ids\)', f'[1] * {count}', fixed_match)
            fixed_match = re.sub(r'\[1\.0\] \* len\(neuron_ids\)', f'[1.0] * {count}', fixed_match)
            fixed_match = re.sub(r'\[0\.0\] \* len\(neuron_ids\)', f'[0.0] * {count}', fixed_match)
            fixed_match = re.sub(r'\[10\] \* len\(neuron_ids\)', f'[10] * {count}', fixed_match)
            fixed_match = re.sub(r'\[0\] \* len\(neuron_ids\)', f'[0] * {count}', fixed_match)
            
            return fixed_match
        
        return full_match
    
    # Match add_neurons_batch calls
    pattern = r'\.add_neurons_batch\(\s*.*?\)'
    content = re.sub(pattern, fix_len_neuron_ids, content, flags=re.DOTALL)
    
    # Fix broken syntax like: positions=[(0, 0, 0, <newline> # NEW REQUIRED...
    content = re.sub(r'positions=\[\(([^)]+),\s*\n\s*#\s*NEW REQUIRED', r'positions=[(\1)],\n            # NEW REQUIRED', content)
    
    # Fix broken positions arrays
    content = re.sub(r'consecutive_fire_limits=\[10\] \* \d+ # Required from genome\)([^,\]]+)\]', 
                     r'consecutive_fire_limits=[10] * 3  # Required from genome\n        )', content)
    
    # Fix malformed positions parameter
    content = re.sub(r'positions=\[.*?# Required from genome\)([^\]]+)\]', 
                     lambda m: f'positions={m.group(1).strip()},', content)
    
    with open(file_path, 'w') as f:
        f.write(content)

def main():
    # Fix the main problematic file
    fix_syntax_errors("tests/npu/test_npu_synaptic_propagation.py")
    print("✅ Fixed syntax errors in test_npu_synaptic_propagation.py")

if __name__ == "__main__":
    main()
