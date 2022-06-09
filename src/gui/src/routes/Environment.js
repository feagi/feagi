import React from "react";
import { useNavigate } from "react-router-dom";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
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
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select an Environment
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 2 }}
      >
        <Item>
          <MenuCard
            image={<GiVintageRobot size={150} />}
            label="Physical Robot"
            onClick={handlePhysicalRobotClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<AiOutlineRobot size={150} />}
            label="Virtual Robot"
            onClick={handleVirtualRobotClick}
          />
        </Item>
      </Stack>
    </>
  );
};

export default ChooseEnvironment;
