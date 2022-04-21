import React from "react";
import Stack from "@mui/material/Stack";
import { GiVintageRobot } from "react-icons/gi";
import { AiOutlineRobot } from "react-icons/ai";
import { Link } from "react-router-dom";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const ChooseEnv = () => {
  return (
    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={2}
    >
      <Item>
        <Link to={"/genome"}>
          <MenuCard
            image={<GiVintageRobot size="xl" />}
            label="Physical Robot"
            info="A physical device with basic I/O and processing power capable of running or interacting with FEAGI"
          />
        </Link>
      </Item>
      <Item>
        <Link to={"/genome"}>
          <MenuCard
            image={<AiOutlineRobot size="xl" />}
            label="Virtual Robot"
            info="A virtual robot existing in a virtual environment (ex: Gazebo) with a defined physics engine capable of interfacing with FEAGI"
          />
        </Link>
      </Item>
    </Stack>
  );
};

export default ChooseEnv;
