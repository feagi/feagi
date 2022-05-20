import React from "react";
import { useNavigate } from "react-router-dom";
import Stack from "@mui/material/Stack";
import { GiVintageRobot } from "react-icons/gi";
import { AiOutlineRobot } from "react-icons/ai";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const ChooseEnvironment = () => {
  let navigate = useNavigate();

  const handlePhysicalRobotClick = () => {
    navigate("/genome/mode");
  };

  const handleVirtualRobotClick = () => {
    navigate("/genome/mode");
  };

  return (
    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={2}
      sx={{ m: 6 }}
    >
      <Item>
        <MenuCard
          image={<GiVintageRobot size={150} />}
          label="Physical Robot"
          onClick={handlePhysicalRobotClick}
          // info="A physical device with basic I/O and processing power capable of running or interacting with FEAGI"
        />
      </Item>
      <Item>
        <MenuCard
          image={<AiOutlineRobot size={150} />}
          label="Virtual Robot"
          onClick={handleVirtualRobotClick}
          // info="A virtual robot existing in a virtual environment (ex: Gazebo) with a defined physics engine capable of interfacing with FEAGI"
        />
      </Item>
    </Stack>
  );
};

export default ChooseEnvironment;
