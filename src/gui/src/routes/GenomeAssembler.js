import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { GiBrainDump } from "react-icons/gi";
import CircularProgress from "@mui/material/CircularProgress";
import Button from "@mui/material/Button";
import Stack from "@mui/material/Stack";
import FeagiAPI from "../services/FeagiAPI";
import { genomeBase } from "../constants/genome";

const GenomeAssembler = (props) => {
  const [loading, setLoading] = useState(false);

  const matchMappings = (mappingData, sensoryData, motorData) => {
    Object.keys(mappingData).forEach((key) => {
      if (key in sensoryData && !(key in motorData)) {
        insertMappingData(mappingData, key, sensoryData);
      } else if (key in motorData && !(key in sensoryData)) {
        insertMappingData(mappingData, key, motorData);
      }
    });
  };

  const insertMappingData = (mappingData, targetKey, corticalAreaData) => {
    Object.keys(corticalAreaData[targetKey]).forEach((subKey) => {
      if (subKey.includes("-dstmap-d")) {
        for (let i = 0; i < mappingData[targetKey].length; i++) {
          corticalAreaData[targetKey][subKey] = {
            [mappingData[targetKey][i]["dstArea"]]: [
              [
                mappingData[targetKey][i]["rule"],
                mappingData[targetKey][i]["morphologyScalar"],
                mappingData[targetKey][i]["pscMultiplier"],
                mappingData[targetKey][i]["plasticity"],
              ],
            ],
          };
        }
      }
    });
  };

  let navigate = useNavigate();

  const generateAndSendGenome = () => {
    setLoading(true);

    let mappingDataCopy = structuredClone(props.definedMappings);
    let sensoryDataCopy = structuredClone(props.definedSensory);
    let motorDataCopy = structuredClone(props.definedMotor);

    matchMappings(mappingDataCopy, sensoryDataCopy, motorDataCopy);

    let blueprintShell = {};
    let combinedDataVals = Object.values({
      ...sensoryDataCopy,
      ...motorDataCopy,
    });

    for (let i = 0; i < combinedDataVals.length; i++) {
      for (const key in combinedDataVals[i]) {
        blueprintShell[key] = combinedDataVals[i][key];
      }
    }

    genomeBase["blueprint"] = blueprintShell;
    FeagiAPI.postGenomeString({ genome: genomeBase });
    navigate("/monitoring");
  };

  return (
    <Stack
      direction="column"
      alignItems="center"
      justifyContent="center"
      spacing={12}
      sx={{ mt: 24 }}
    >
      <GiBrainDump size={300} />
      {loading ? (
        <CircularProgress size="150px" />
      ) : (
        <Button variant="contained" onClick={generateAndSendGenome}>
          CREATE GENOME AND START
        </Button>
      )}
    </Stack>
  );
};

export default GenomeAssembler;
