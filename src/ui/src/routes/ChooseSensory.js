import React, { useState } from "react";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { FaRuler } from "react-icons/fa";
import { TiWaves, TiBatteryFull } from "react-icons/ti";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const ChooseSensory = () => {
  const [selectedSensory, setSelectedSensory] = useState([]);

  const handleClick = (e) => {
    e.preventDefault();
    console.log(e);
  };

  return (
    <>
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
          <MenuCard image={<FaRuler size={200} />} label="Proximity" />
        </Item>
        <Item>
          <MenuCard image={<TiBatteryFull size={200} />} label="Battery" />
        </Item>
      </Stack>
    </>
  );
};

export default ChooseSensory;
