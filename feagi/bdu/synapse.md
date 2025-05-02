# SynapseManager Documentation

## Overview

`SynapseManager` is a high-performance, memory-efficient data structure designed to store and manage synapses in a large-scale **spiking neural network (SNN)**. It supports **sparse synaptic connectivity**, **plasticity modeling**, and **parallel computation on CPUs and GPUs**.

This document provides an in-depth explanation of `synapse.py`, detailing its design choices, data structures, and functionality.

---

## Features

- **Sparse Synapse Representation**: Uses a compressed sparse row (CSR) format to store synapses efficiently.
- **Plastic & Non-Plastic Synapses**: Distinguishes between synapses that undergo plasticity and those that do not.
- **Parallel Computation**: Optimized using `numba` for parallel updates.
- **Memory Efficient**: Stores plasticity attributes **only for plastic synapses**.
- **Flexible Synaptic Plasticity**: Supports **Short-Term Plasticity (STP)** and **Long-Term Potentiation/Depression (LTP/LTD)**.

---

## Data Structure

### **Core Synapse Storage (CSR Format)**

Each neuron maintains a **sparse edge list** of its outgoing synapses:

| Data Structure    | Type         | Description                                               |
| ----------------- | ------------ | --------------------------------------------------------- |
| `synapse_row_ptr` | `np.uint32`  | Index pointers into `synapse_col_idx` array (CSR format). |
| `synapse_col_idx` | `np.uint32`  | Stores post-synaptic neuron indices.                      |
| `synapse_weights` | `np.float32` | Strength of each synapse.                                 |
| `synapse_count`   | `np.uint32`  | Number of outgoing synapses per neuron.                   |

---

### **Plasticity Attributes (Stored Only for Plastic Synapses)**

These attributes are **allocated only when plasticity is enabled**:

| Attribute              | Type         | Description                              |
| ---------------------- | ------------ | ---------------------------------------- |
| `plastic_synapse_mask` | `bool`       | Marks whether a synapse is plastic.      |
| `plasticity_type`      | `np.uint8`   | 0: None, 1: STP, 2: LTP/LTD.             |
| `plasticity_coeff`     | `np.float32` | Coefficient controlling synaptic change. |
| `plasticity_decay`     | `np.float32` | Decay rate of plasticity effect.         |
| `activity_factor`      | `np.float32` | Multiplier for synaptic activity.        |
| `scaling_exponent`     | `np.float32` | Nonlinear scaling factor for plasticity. |

---

## **Class: **``

### **Initialization (**``**)**

```python
synapses = SynapseManager(num_neurons=10, max_synapses_per_neuron=1024)
```

| Parameter                 | Type  | Description                             |
| ------------------------- | ----- | --------------------------------------- |
| `num_neurons`             | `int` | Total number of neurons in the network. |
| `max_synapses_per_neuron` | `int` | Upper bound on synapses per neuron.     |

---

### **Adding Synapses (**``**)**

```python
synapses.add_synapse(pre_neuron=0, post_neuron=2, weight=100, plasticity_type=1, plasticity_coeff=1.5)
```

| Parameter          | Type      | Description                                 |
| ------------------ | --------- | ------------------------------------------- |
| `pre_neuron`       | `int`     | ID of the pre-synaptic neuron.              |
| `post_neuron`      | `int`     | ID of the post-synaptic neuron.             |
| `weight`           | `float32` | Initial synaptic weight.                    |
| `plasticity_type`  | `int`     | 0: None, 1: STP, 2: LTP/LTD.                |
| `plasticity_coeff` | `float32` | Coefficient controlling plasticity changes. |
| `plasticity_decay` | `float32` | Decay factor for plastic synapses.          |
| `activity_factor`  | `float32` | Influence of neural activity.               |
| `scaling_exponent` | `float32` | Exponent for nonlinear scaling.             |

This method ensures **no overwrites** and maintains **proper indexing**.

---

### **Finalizing Synapses (**``**)**

```python
synapses.finalize_synapses()
```

- Computes **CSR row pointers** to efficiently retrieve synapses.

---

### **Retrieving Downstream Neurons (**``**)**

```python
synapses.get_downstream_neurons(0)
```

- Returns a **list of post-synaptic neurons** for a given neuron.

---

### **Updating Plasticity (**``**)**

```python
synapses.update_plasticity(dt=1)
```

- Modifies **synaptic weights** based on plasticity parameters.
- Uses **Numba for parallelized execution**.

---

## **Plasticity Update Logic (**``**)**

```python
@njit(parallel=True, fastmath=True)
def _update_plasticity(plasticity_type, plasticity_coeff, plasticity_decay, activity_factor, scaling_exponent, synapse_weights, num_synapses, dt):
    min_bound = np.float32(0.0)
    max_bound = np.float32(255.0)

    for i in prange(num_synapses):
        if plasticity_type[i] == 1:  # STP
            new_weight = synapse_weights[i] * (plasticity_coeff[i] ** scaling_exponent[i]) * activity_factor[i] * (plasticity_decay[i] ** dt)
            synapse_weights[i] = min(max(new_weight, min_bound), max_bound)
        elif plasticity_type[i] == 2:  # LTP/LTD
            new_weight = synapse_weights[i] + (plasticity_coeff[i] ** scaling_exponent[i]) * activity_factor[i] * (plasticity_decay[i] ** dt) * synapse_weights[i]
            synapse_weights[i] = min(max(new_weight, min_bound), max_bound)
```

### **Key Features:**

- **Parallel execution** with `numba.prange()`.
- **Clamps weights** to `[0, 255]`.
- **Differentiates STP & LTP/LTD.**

---

## **Example Usage**

```python
# Initialize
synapses = SynapseManager(num_neurons=10)

# Add non-plastic synapse
synapses.add_synapse(0, 2, weight=100, plasticity_type=0)

# Add plastic synapse
synapses.add_synapse(1, 3, weight=150, plasticity_type=1, plasticity_coeff=1.5)

# Finalize connections
synapses.finalize_synapses()

# Get downstream neurons
print(synapses.get_downstream_neurons(0))  # Output: [2]

# Run plasticity update
synapses.update_plasticity(dt=1)
```

---

## **Performance Considerations**

- **Memory Efficiency**: Uses **bit-packed storage** to minimize RAM usage.
- **Parallelized Updates**: Numba accelerates computations for large-scale models.
- **Scalability**: Supports **hundreds of millions of neurons** efficiently.

---

## **Future Enhancements**

- **Synapse Pruning**: Ability to remove weak synapses.
- **GPU Acceleration**: Extend parallelism to CUDA for further speedups.
- **Dynamic Synapse Growth**: Enable new synapse formation during simulation.

---

## **Conclusion**

`SynapseManager` is a **highly efficient**, **scalable**, and **parallelizable** neural network synapse manager. Its **sparse storage format**, **plasticity modeling**, and **performance optimizations** make it well-suited for large-scale **neuromorphic computing and AI applications**.

