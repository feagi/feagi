import React, { useState } from "react";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MenuDialog from "../components/MenuDialog";

const Mapping = (props) => {
  console.log(props);
  // convert props.definedSensory, props.definedMotor to arrays
  // so that they work with .map

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
        {props.definedSensory.map((item) => (
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
        {props.definedMotor.map((item) => (
          <Item key={item}>
            <MenuDialog label={item} mode="map" />
          </Item>
        ))}
      </Stack>
    </>
  );
};

export default Mapping;
