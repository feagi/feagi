import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import Fab from "@mui/material/Fab";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MenuDialog from "../components/MenuDialog";

const Mapping = (props) => {
  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/genome/assemble");
  };

  return (
    <>
      <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
        Cortical Area Mapping
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={12}
        sx={{ m: 1 }}
      >
        {Object.keys(props.definedSensory).map((item) => (
          <Item key={item}>
            <MenuDialog
              definedMappings={props.definedMappings}
              setDefinedMappings={props.setDefinedMappings}
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
        spacing={12}
        sx={{ m: 24 }}
      >
        {Object.keys(props.definedMotor).map((item) => (
          <Item key={item}>
            <MenuDialog label={item} mode="map" />
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
        <Tooltip title="Next">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!props.definedMotor}
            >
              <ArrowForwardIcon onClick={handleNext} />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default Mapping;
