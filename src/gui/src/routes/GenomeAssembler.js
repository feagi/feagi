import React, { useState } from "react";
import Button from "@mui/material/Button";
import Typography from "@mui/material/Typography";
import FeagiAPI from "../services/FeagiAPI";

const GenomeAssembler = (props) => {
  // add title, button, some sort of progress/loading indicator
  console.log(props);

  let genomeBase = {
    version: "2.0",
    max_burst_count: 3,
    burst_delay: 0.1,
    evolution_burst_count: 50,
    ipu_idle_threshold: 1000,
    neuron_morphologies: {
      placeholder: {},
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

  const matchMappings = (mappingData, sensoryData, motorData) => {
    Object.keys(mappingData).forEach((key) => {
      if (key in sensoryData && !(key in motorData)) {
        console.log("ONLY IN SENSORY!!!");
        insertMappingData(mappingData, key, sensoryData);
        console.log(sensoryData);
      } else if (key in motorData && !(key in sensoryData)) {
        console.log("ONLY IN MOTOR!!!");
        insertMappingData(mappingData, key, motorData);
        console.log(motorData);
      }
    });
  };

  const insertMappingData = (mappingData, targetKey, corticalAreaData) => {
    Object.keys(corticalAreaData[targetKey]).forEach((subKey) => {
      if (subKey.includes("-dstmap-d")) {
        for (let i = 0; i < mappingData[targetKey].length; i++) {
          corticalAreaData[targetKey][subKey] = {
            [mappingData[targetKey][i]["dstArea"]]: [
              [mappingData[targetKey][i]["rule"], [1, 1, 1], 1, false],
            ],
          };
        }
      }
    });
  };

  const generateGenome = (sensoryData, motorData, genomeBase) => {
    let blueprintShell = {};
    let combinedDataVals = Object.values({ ...sensoryData, ...motorData });
    // loop through combined values and add them to blueprintShell
    // then genomeBase["blueprint"] = blueprintShell
  };

  let mappingDataCopy = structuredClone(props.definedMappings);
  let sensoryDataCopy = structuredClone(props.definedSensory);
  let motorDataCopy = structuredClone(props.definedMotor);

  matchMappings(mappingDataCopy, sensoryDataCopy, motorDataCopy);
  generateGenome(sensoryDataCopy, motorDataCopy, genomeBase);

  return <div>GenomeAssembler</div>;
};

export default GenomeAssembler;
