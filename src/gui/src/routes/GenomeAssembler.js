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
      console.log("IT IS IN BOTH!!!");
    } else if (key in props.definedSensory && !(key in props.definedMotor)) {
      console.log("IT IS ONLY IN SENSORY!!!");

      // create a function for performing this operation
      Object.keys(props.definedSensory[key]).forEach((subKey) => {
        if (subKey.includes("-dstmap-d")) {
          for (let i = 0; i < props.definedMappings[key].length; i++) {
            props.definedSensory[key][subKey] = {
              [props.definedMappings[key][i]["dstArea"]]: [
                [
                  props.definedMappings[key][i]["rule"],
                  props.definedMappings[key][i]["info"][
                    Object.keys(props.definedMappings[key][i]["info"])[0]
                  ],
                  1,
                  false,
                ],
              ],
            };
          }
        }
      });
    } else if (key in props.definedMotor && !(key in props.definedSensory)) {
      console.log("IT IS ONLY IN MOTOR!!!");

      // create a function for performing this operation
      Object.keys(props.definedMotor[key]).forEach((subKey) => {
        if (subKey.includes("-dstmap-d")) {
          for (let i = 0; i < props.definedMappings[key].length; i++) {
            props.definedMotor[key][subKey] = {
              [props.definedMappings[key][i]["dstArea"]]: [
                [
                  props.definedMappings[key][i]["rule"],
                  props.definedMappings[key][i]["info"][
                    Object.keys(props.definedMappings[key][i]["info"])[0]
                  ],
                  1,
                  false,
                ],
              ],
            };
          }
        }
      });
    }
  });

  console.log(props.definedSensory);
  console.log(props.definedMotor);

  const insertMappingData = (mappingData, sensoryData, motorData) => {};

  const generateGenome = (sensoryData, motorData) => {};

  return <div>GenomeAssembler</div>;
};

export default GenomeAssembler;
