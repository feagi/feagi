import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import Paper from "@mui/material/Paper";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import Fab from "@mui/material/Fab";
import Stack from "@mui/material/Stack";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import { FaRuler } from "react-icons/fa";
import { TiWaves } from "react-icons/ti";
import { MdBatteryCharging80 } from "react-icons/md";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import MenuDialog from "../components/MenuDialog";

const Sensory = (props) => {
  const [selectedSensory, setSelectedSensory] = useState([]);

  const handleClick = (e, src) => {
    if (!selectedSensory.includes(src)) {
      let updatedSensory = [...selectedSensory, src];
      setSelectedSensory(updatedSensory);
    } else {
      let filteredSensory = selectedSensory.filter((item) => item !== src);
      setSelectedSensory(filteredSensory);
    }
  };

  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/brain/motor");
  };

  return (
    <>
      <Typography variant="h4" align="center" sx={{ p: 4 }} component="div">
        Choose Sensory Abilities
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
            image={<TiWaves size={150} />}
            label="Infrared"
            onClick={handleClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<FaRuler size={150} />}
            label="Proximity"
            onClick={handleClick}
          />
        </Item>
        <Item>
          <MenuCard
            image={<MdBatteryCharging80 size={150} />}
            label="Battery"
            onClick={handleClick}
          />
        </Item>
      </Stack>
      <Typography variant="h4" align="center" sx={{ p: 2 }} component="div">
        Selected Sensory
      </Typography>
      <Paper
        elevation={1}
        sx={{ mx: "30rem", mb: "1rem", height: "75px", p: 2 }}
      >
        <Stack
          direction="row"
          alignItems="center"
          justifyContent="center"
          spacing={2}
          sx={{ m: 1 }}
        >
          {selectedSensory.map((item) => (
            <Item key={item}>
              <MenuDialog
                definedSensory={props.definedSensory}
                setDefinedSensory={props.setDefinedSensory}
                label={item}
                mode="define"
                type="sensory"
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
              disabled={!props.definedSensory}
            >
              <ArrowForwardIcon onClick={handleNext} />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default Sensory;
