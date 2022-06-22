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
  let navigate = useNavigate();

  const matchMappings = (mappingData, corticalAreaData) => {
    Object.keys(mappingData).forEach((key) => {
      if (key in corticalAreaData) {
        insertMappingData(mappingData, key, corticalAreaData);
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

  // const sleep = (ms) => {
  //   return new Promise((resolve) => setTimeout(resolve, ms));
  // };

  // const burstCountPoller = async () => {
  //   let bursting = false;
  //   while (!bursting) {
  //     const burstCount = await FeagiAPI.getBurstCount();
  //     if (burstCount > 0) {
  //       bursting = true;
  //       navigate("/monitoring");
  //     }
  //     await sleep(2000);
  //   }
  // };

  const generateAndSendGenome = () => {
    setLoading(true);

    let mappingDataCopy = structuredClone(props.definedMappings);
    let areaDataCopy = structuredClone(props.definedAreas);

    matchMappings(mappingDataCopy, areaDataCopy);

    let blueprintShell = {};
    let areaDataVals = Object.values(areaDataCopy);

    for (let i = 0; i < areaDataVals.length; i++) {
      for (const key in areaDataVals[i]) {
        blueprintShell[key] = areaDataVals[i][key];
      }
    }

    genomeBase["blueprint"] = blueprintShell;
    FeagiAPI.postGenomeString({ genome: genomeBase });
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
