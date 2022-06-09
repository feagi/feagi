export const genomeBase = {
  version: "2.0",
  max_burst_count: 3,
  burst_delay: 0.1,
  evolution_burst_count: 50,
  ipu_idle_threshold: 1000,
  neuron_morphologies: {
    block_to_block: {
      vectors: [[0, 0, 0]],
    },
    decrease_filter_diagonal: {
      vectors: [[0, 1, 1]],
    },
    decrease_filter_diagonal2: {
      vectors: [[0, 2, 1]],
    },
    increase_filter_diagonal: {
      vectors: [[0, 1, -1]],
    },
    increase_filter_diagonal2: {
      vectors: [[0, 2, -1]],
    },
    y_consolidator: {
      patterns: [
        ["*", "*", "*"],
        ["*", "?", "*"],
      ],
    },
    "lateral_+x": {
      vectors: [[1, 0, 0]],
    },
    "lateral_-x": {
      vectors: [[-1, 0, 0]],
    },
    "lateral_+y": {
      vectors: [[0, 1, 0]],
    },
    "lateral_-y": {
      vectors: [[0, -1, 0]],
    },
    "lateral_+z": {
      vectors: [[0, 0, 1]],
    },
    "lateral_-z": {
      vectors: [[0, 0, -1]],
    },
    one_to_all: {
      patterns: [
        [1, 1, 1],
        ["*", "*", "*"],
      ],
    },
    all_to_one: {
      patterns: [
        ["*", "*", "*"],
        [1, 1, 1],
      ],
    },
    "to_block_[5, 7, 4]": {
      patterns: [
        ["*", "*", "*"],
        [5, 7, 4],
      ],
    },
    expander_x: {
      functions: true,
    },
    reducer_x: {
      functions: true,
    },
    randomizer: {
      functions: true,
    },
    lateral_pairs_x: {
      functions: true,
    },
  },
  species: {
    parents: {},
    species_id: "",
    class: "toy",
    brand: "gazebo",
    model: "smart_car",
  },
  blueprint: {},
};
