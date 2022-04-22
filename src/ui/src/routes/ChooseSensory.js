import React, { useState } from "react";
import Button from "@mui/material/Button";
import Paper from "@mui/material/Paper";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { FaRuler } from "react-icons/fa";
import { TiWaves, TiBatteryFull } from "react-icons/ti";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const ChooseSensory = () => {
  const [selectedSensory, setSelectedSensory] = useState([]);

  const handleClick = (e, src) => {
    if (!selectedSensory.includes(src)) {
      let updatedSensory = [...selectedSensory, src];
      setSelectedSensory(updatedSensory);
    } else {
      let filteredSensory = selectedSensory.filter((item) => item !== src);
      setSelectedSensory(filteredSensory);
    }
  };

  return (
    <>
      <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
        Choose Sensory Abilities
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
            image={<TiWaves size={200} />}
            label="Infrared"
            onClick={handleClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<FaRuler size={200} />}
            label="Proximity"
            onClick={handleClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<TiBatteryFull size={200} />}
            label="Battery"
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

export default ChooseSensory;
