import React, { useState } from "react";
import Button from "@mui/material/Button";
import Typography from "@mui/material/Typography";
import FeagiAPI from "../services/FeagiAPI";

const GenomeAssembler = (props) => {
  // don't forget to insert additional genome data (version, morphologies, etc.)

  // default values for post-synaptic current multiplier + plasticity flag?
  // or add user inputs for these in the cortical mapping form?

  // add title, button, some sort of progress/loading indicator

  const [genomeString, setGenomeString] = useState({});

  console.log(props.definedSensory);
  console.log(props.definedMotor);

  Object.keys(props.definedMappings).forEach((key) => {
    if (key in props.definedSensory && key in props.definedMotor) {
      console.log("IN BOTH!!!");

      insertMappingData(props.definedMappings, key, props.definedSensory);
      insertMappingData(props.definedMappings, key, props.definedMotor);
    } else if (key in props.definedSensory && !(key in props.definedMotor)) {
      console.log("ONLY IN SENSORY!!!");

      insertMappingData(props.definedMappings, key, props.definedSensory);

      // create a function for performing this operation
      // Object.keys(props.definedSensory[key]).forEach((subKey) => {
      //   if (subKey.includes("-dstmap-d")) {
      //     for (let i = 0; i < props.definedMappings[key].length; i++) {
      //       props.definedSensory[key][subKey] = {
      //         [props.definedMappings[key][i]["dstArea"]]: [
      //           [
      //             props.definedMappings[key][i]["rule"],
      //             props.definedMappings[key][i]["info"][
      //               Object.keys(props.definedMappings[key][i]["info"])[0]
      //             ],
      //             1,
      //             false,
      //           ],
      //         ],
      //       };
      //     }
      //   }
      // });
    } else if (key in props.definedMotor && !(key in props.definedSensory)) {
      console.log("ONLY IN MOTOR!!!");

      insertMappingData(props.definedMappings, key, props.definedMotor);

      // create a function for performing this operation
      // Object.keys(props.definedMotor[key]).forEach((subKey) => {
      //   if (subKey.includes("-dstmap-d")) {
      //     for (let i = 0; i < props.definedMappings[key].length; i++) {
      //       props.definedMotor[key][subKey] = {
      //         [props.definedMappings[key][i]["dstArea"]]: [
      //           [
      //             props.definedMappings[key][i]["rule"],
      //             props.definedMappings[key][i]["info"][
      //               Object.keys(props.definedMappings[key][i]["info"])[0]
      //             ],
      //             1,
      //             false,
      //           ],
      //         ],
      //       };
      //     }
      //   }
      // });
    }
  });

  console.log(props.definedSensory);
  console.log(props.definedMotor);

  const matchMappings = (mappingData, sensoryData, motorData) => {
    Object.keys(mappingData).forEach((key) => {
      if (key in sensoryData && key in motorData) {
        insertMappingData(mappingData, key, sensoryData);
        insertMappingData(mappingData, key, motorData);
      } else if (key in sensoryData && !(key in motorData)) {
        insertMappingData(mappingData, key, sensoryData);
      } else if (key in motorData && !(key in sensoryData)) {
        insertMappingData(mappingData, key, motorData);
      }
    });
  };

  const insertMappingData = (mappingData, targetKey, corticalAreaData) => {
    Object.keys(mappingData[targetKey]).forEach((subKey) => {
      if (subKey.includes("-dstmap-d")) {
        for (let i = 0; i < mappingData[targetKey].length; i++) {
          corticalAreaData[targetKey][subKey] = {
            [mappingData[targetKey][i]["dstArea"]]: [
              [
                mappingData[targetKey][i]["rule"],
                mappingData[targetKey][i]["info"][
                  Object.keys(mappingData[targetKey][i]["info"])[0]
                ],
                1,
                false,
              ],
            ],
          };
        }
      }
    });
  };

  const generateGenome = (sensoryData, motorData) => {};

  return <div>GenomeAssembler</div>;
};

export default GenomeAssembler;
