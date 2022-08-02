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

  const handleRobotSelection = (type) => {
      FeagiAPI.postRobotModel({
        robot_model: type,
        robot_model_path: "./src/evo/defaults/robot/"
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
              label="Taffy-bot"
              onClick={() => handleRobotSelection("taffy_robot.sdf")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<Img src={require('../assets/cloud_bot.png')} width="150" height="150" />}
              label="Cloud-bot"
              onClick={() => handleRobotSelection("cloud_robot.sdf")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<Img src={require('../assets/3w_robot.png')} width="150" height="150" />}
              label="3W Robot"
              onClick={() =>
                handleRobotSelection("3w_robot.sdf")
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
