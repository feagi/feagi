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
import {Img} from 'react-image'

const DefaultRobots = (props) => {
  let navigate = useNavigate();

  const handleRobotSelection = (type, path) => {
      FeagiAPI.postRobotModel({
        robot_sdf_file_name: JSON.parse(type),
        robot_sdf_file_name_path: JSON.parse(path)
       });
    navigate("/genome/mode");
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select a Robot
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
              image={<Img src={require('../assets/taffy_bot.png')} width="150" height="150" />}
              label="Smart Car"
              onClick={() => handleRobotSelection('"taffy_bot.sdf"', '"/robots/smart_car/"')}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<Img src={require('../assets/stick-bot.png')} width="150" height="150" />}
              label="Stick-bot"
              onClick={() => handleRobotSelection('"stick-bot.sdf"', '"/robots/stick-bot/"')}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<Img src={require('../assets/drone.png')} width="150" height="150" />}
              label="Drone"
              onClick={() =>
                handleRobotSelection('"drone.sdf"', '"/robots/drone/"')
              }
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default DefaultRobots;
