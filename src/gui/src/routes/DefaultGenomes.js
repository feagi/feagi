import React, { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { AiOutlineCar } from "react-icons/ai";
import { BiDna } from "react-icons/bi";
import { ImBrightnessContrast } from "react-icons/im";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import FeagiAPI from "../services/FeagiAPI";

const DefaultGenomes = (props) => {
  let navigate = useNavigate();

  const handleGenomeSelection = (type) => {
    FeagiAPI.postGenomeString(
      props.defaultGenomeData[type]
    );
    navigate("/monitoring");
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select a Sample Genome
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 2 }}
      >
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<BiDna size={150} />}
              label="Base Genome"
              onClick={() => handleGenomeSelection("static_genome")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<AiOutlineCar size={150} />}
              label="Motor Babbler"
              onClick={() => handleGenomeSelection("motor_babble_genome")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<ImBrightnessContrast size={150} />}
              label="Brightness Detection"
              onClick={() =>
                handleGenomeSelection("brightness_change_detection")
              }
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default DefaultGenomes;
