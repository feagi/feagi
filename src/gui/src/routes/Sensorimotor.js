import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import Fab from "@mui/material/Fab";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import { FaRuler } from "react-icons/fa";
import { TiWaves } from "react-icons/ti";
import { MdBatteryCharging80 } from "react-icons/md";
import { GiGears } from "react-icons/gi";
import { BsGearWide } from "react-icons/bs";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";

const Sensorimotor = (props) => {
  const [numberOfMotors, setNumberOfMotors] = useState(null);
  const [numberOfSensors, setNumberOfSensors] = useState(null);

  const handleSensoryClick = (e, src) => {
    if (!props.selectedSensory.includes(src)) {
      let updatedSensory = [...props.selectedSensory, src];
      props.setSelectedSensory(updatedSensory);
    } else {
      let filteredSensory = props.selectedSensory.filter(
        (item) => item !== src
      );
      props.setSelectedSensory(filteredSensory);
    }
  };

  const handleMotorClick = (e, src) => {
    if (!props.selectedMotor.includes(src)) {
      let updatedMotor = [...props.selectedMotor, src];
      props.setSelectedMotor(updatedMotor);
    } else {
      let filteredMotor = props.selectedMotor.filter((item) => item !== src);
      props.setSelectedMotor(filteredMotor);
    }
  };

  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/brain/editor");
  };

  const handleBack = () => {
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
        Select Sensory
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ mb: 2 }}
      >
        <Item>
          <MenuCard
            image={<TiWaves size={150} />}
            label="Infrared"
            onClick={handleSensoryClick}
            changeColorOnClick={true}
          />
        </Item>
        <Item>
          <MenuCard
            image={<FaRuler size={150} />}
            label="Proximity"
            onClick={handleSensoryClick}
            changeColorOnClick={true}
          />
        </Item>
        <Item>
          <MenuCard
            image={<MdBatteryCharging80 size={150} />}
            label="Battery"
            onClick={handleSensoryClick}
            changeColorOnClick={true}
          />
        </Item>
      </Stack>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 12 }}
        component="div"
      >
        Select Motor
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ mb: 18 }}
      >
        <Item>
          <MenuCard
            image={<GiGears size={150} />}
            label="Motor"
            onClick={handleMotorClick}
            changeColorOnClick={true}
          />
        </Item>
        <Item>
          <MenuCard
            image={<BsGearWide size={150} />}
            label="Servo"
            onClick={handleMotorClick}
            changeColorOnClick={true}
          />
        </Item>
      </Stack>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ mb: 8 }}
      >
        <Tooltip title="Back">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              onClick={handleBack}
            >
              <ArrowBackIcon />
            </Fab>
          </span>
        </Tooltip>
        <Tooltip title="Next">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              onClick={handleNext}
            >
              <ArrowForwardIcon />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default Sensorimotor;
