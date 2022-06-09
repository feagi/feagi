import React from "react";
import { useNavigate } from "react-router-dom";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import Fab from "@mui/material/Fab";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MappingDialog from "../components/MappingDialog";

const Mapping = (props) => {
  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/genome/assemble");
  };

  const handleBack = () => {
    navigate("/brain/editor");
  };

  const getAvailableMappings = (areaData) => {
    let availableAreaNames = [];
    for (const definedArea in areaData) {
      for (const geneSample in areaData[definedArea]) {
        availableAreaNames.push(geneSample.slice(9, 15));
        break;
      }
    }
    return availableAreaNames;
  };

  const availableMappingAreas = getAvailableMappings(props.definedAreas);

  return (
    <>
      <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
        Cortical Area Mapping
      </Typography>
      <Stack
        direction="column"
        alignItems="center"
        justifyContent="center"
        spacing={12}
        sx={{ mt: 12, mb: 24 }}
      >
        {Object.keys(props.definedAreas).map((item) => (
          <Item key={item}>
            <MappingDialog
              definedMappings={props.definedMappings}
              setDefinedMappings={props.setDefinedMappings}
              availableMappingAreas={availableMappingAreas}
              defaultMorphologyScalarX={props.defaultMorphologyScalarX}
              defaultMorphologyScalarY={props.defaultMorphologyScalarY}
              defaultMorphologyScalarZ={props.defaultMorphologyScalarZ}
              defaultPscMultiplier={props.defaultPscMultiplier}
              defaultPlasticityFlag={props.defaultPlasticityFlag}
              defaultSynapseRules={props.defaultSynapseRules}
              label={item}
              mode="map"
            />
          </Item>
        ))}
      </Stack>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ mb: 8 }}
      >
        <Tooltip title="Back">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!props.definedMappings}
              onClick={handleBack}
            >
              <ArrowBackIcon />
            </Fab>
          </span>
        </Tooltip>
        <Tooltip title="Next">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!props.definedMappings}
              onClick={handleNext}
            >
              <ArrowForwardIcon />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default Mapping;
