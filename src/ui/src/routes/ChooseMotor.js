import React, { useState } from "react";
import Button from "@mui/material/Button";
import Paper from "@mui/material/Paper";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { GiGears } from "react-icons/gi";
import { BsGearWide } from "react-icons/bs";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const ChooseMotor = () => {
  const [selectedMotor, setSelectedMotor] = useState([]);

  const handleClick = (e, src) => {
    if (!selectedMotor.includes(src)) {
      let updatedMotor = [...selectedMotor, src];
      setSelectedMotor(updatedMotor);
    } else {
      let filteredMotor = selectedMotor.filter((item) => item !== src);
      setSelectedMotor(filteredMotor);
    }
  };

  return (
    <>
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
        Selected
      </Typography>
      <Paper elevation={1} sx={{ mx: "30rem", mb: "5rem", p: 4 }}>
        <Stack
          direction="row"
          alignItems="center"
          justifyContent="center"
          spacing={2}
          sx={{ m: 2 }}
        >
          {selectedSensory.map((item) => (
            <Item key={item}>
              <Button variant="contained">{item}</Button>
            </Item>
          ))}
        </Stack>
      </Paper>
    </>
  );
};

export default ChooseMotor;
