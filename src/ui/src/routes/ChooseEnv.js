import React from "react";
import Grid from "@mui/material/Grid";
import Box from "@mui/material/Box";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import { GiVintageRobot } from "react-icons/gi";
import { AiOutlineRobot } from "react-icons/ai";
import { Link } from "react-router-dom";

const ChooseEnv = () => {
  return (
    <Box sx={{ flexGrow: 1 }}>
      <Grid
        container
        spacing={0}
        direction="row"
        alignItems="center"
        justifyContent="center"
      >
        <Grid item xs={4}>
          <Item>
            <Link to={"/genome"}>
              <MenuCard
                image={<GiVintageRobot size="xl" />}
                label="Physical Robot"
                info="A physical device with basic I/O and processing power capable of running or interacting with FEAGI"
              />
            </Link>
          </Item>
        </Grid>
        <Grid item xs={4}>
          <Item>
            <Link to={"/genome"}>
              <MenuCard
                image={<AiOutlineRobot size="xl" />}
                label="Virtual Robot"
                info="A virtual robot existing in a virtual environment (ex: Gazebo) with a defined physics engine capable of interfacing with FEAGI"
              />
            </Link>
          </Item>
        </Grid>
      </Grid>
    </Box>
  );
};

export default ChooseEnv;
