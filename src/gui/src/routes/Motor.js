import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import Fab from "@mui/material/Fab";
import Paper from "@mui/material/Paper";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import { GiGears } from "react-icons/gi";
import { BsGearWide } from "react-icons/bs";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import MenuDialog from "../components/MenuDialog";

const Motor = (props) => {
  const [selectedMotor, setSelectedMotor] = useState([]);

  const handleClick = (e, src) => {
    if (!selectedMotor.includes(src)) {
      let updatedMotor = [...selectedMotor, src];
      setSelectedMotor(updatedMotor);
    } else {
      let filteredMotor = selectedMotor.filter((item) => item !== src);
      setSelectedMotor(filteredMotor);
    }
  };

  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/brain/mapping");
  };

  return (
    <>
      <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
        Choose Motor Abilities
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 6 }}
      >
        <Item>
          <MenuCard
            image={<GiGears size={150} />}
            label="Motor"
            onClick={handleClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<BsGearWide size={150} />}
            label="Servo"
            onClick={handleClick}
          />
        </Item>
      </Stack>
      <Typography variant="h4" align="center" sx={{ p: 2 }} component="div">
        Selected Motor
      </Typography>
      <Paper
        elevation={1}
        sx={{ mx: "30rem", mb: "5rem", height: "75px", p: 2 }}
      >
        <Stack
          direction="row"
          alignItems="center"
          justifyContent="center"
          spacing={2}
          sx={{ m: 1 }}
        >
          {selectedMotor.map((item) => (
            <Item key={item}>
              <MenuDialog
                definedMotor={props.definedMotor}
                setDefinedMotor={props.setDefinedMotor}
                defaultCorticalGenes={props.defaultCorticalGenes}
                label={item}
                mode="define"
                type="motor"
              />
            </Item>
          ))}
        </Stack>
      </Paper>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ mb: 8 }}
      >
        <Tooltip title="Next">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!props.definedMotor}
            >
              <ArrowForwardIcon onClick={handleNext} />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default Motor;
