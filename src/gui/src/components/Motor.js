import React, { useState } from "react";
import Paper from "@mui/material/Paper";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { GiGears } from "react-icons/gi";
import { BsGearWide } from "react-icons/bs";
import Item from "./Item";
import MenuCard from "./MenuCard";
import MenuDialog from "./MenuDialog";
import MotorContext from "../contexts/MotorContext";

const Motor = ({ setDefinedMotor }) => {
  const [selectedMotor, setSelectedMotor] = useState([]);
  const [finalizedMotor, setFinalizedMotor] = useState({});

  const handleClick = (e, src) => {
    if (!selectedMotor.includes(src)) {
      let updatedMotor = [...selectedMotor, src];
      setSelectedMotor(updatedMotor);
      setDefinedMotor(updatedMotor);
    } else {
      let filteredMotor = selectedMotor.filter((item) => item !== src);
      setSelectedMotor(filteredMotor);
      setDefinedMotor(filteredMotor);
    }
  };

  return (
    <>
      <MotorContext.Provider value={finalizedMotor}>
        <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
          Choose Motor Abilities
        </Typography>
        <Stack
          direction="row"
          alignItems="center"
          justifyContent="center"
          spacing={2}
          sx={{ m: 6 }}
        >
          <Item>
            <MenuCard
              image={<GiGears size={200} />}
              label="Motor"
              onClick={handleClick}
            />
          </Item>
          <Item>
            <MenuCard
              image={<BsGearWide size={200} />}
              label="Servo"
              onClick={handleClick}
            />
          </Item>
        </Stack>
        <Typography variant="h4" align="center" sx={{ p: 2 }} component="div">
          Selected Motor
        </Typography>
        <Paper
          elevation={1}
          sx={{ mx: "30rem", mb: "5rem", height: "75px", p: 2 }}
        >
          <Stack
            direction="row"
            alignItems="center"
            justifyContent="center"
            spacing={2}
            sx={{ m: 1 }}
          >
            {selectedMotor.map((item) => (
              <Item key={item}>
                <MenuDialog label={item} type="cortical" />
              </Item>
            ))}
          </Stack>
        </Paper>
      </MotorContext.Provider>
    </>
  );
};

export default Motor;
