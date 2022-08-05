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
    navigate("/robot/physical");
  };

  const handleVirtualRobotClick = () => {
    navigate("/map/defaults");
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select an Environment Type
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
            label="Physical"
            onClick={handlePhysicalRobotClick}
            changeColorOnClick={false}
            grayedOut={false}
          />
        </Item>
        <Item>
          <MenuCard
            image={<AiOutlineRobot size={150} />}
            label="Virtual"
            onClick={handleVirtualRobotClick}
          />
        </Item>
      </Stack>
    </>
  );
};

export default ChooseEnvironment;
