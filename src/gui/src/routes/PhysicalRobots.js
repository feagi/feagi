import React, { useState } from "react";
import { GrRaspberry } from "react-icons/gr";
import { SiMicrobit } from "react-icons/si";
import Dialog from "@mui/material/Dialog";
import DialogContent from "@mui/material/DialogContent";
import DialogContentText from "@mui/material/DialogContentText";
import DialogTitle from "@mui/material/DialogTitle";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import DialogActions from '@mui/material/DialogActions';
import Button from '@mui/material/Button';
import { useNavigate } from "react-router-dom";


const PhysicalRobots = () => {
  const [freenoveDialogOpen, setFreenoveDialogOpen] = useState(false);
  const [microbitDialogOpen, setMicrobitDialogOpen] = useState(false);
  let navigate = useNavigate();

  const handleFreenoveDialogOpen = () => {
    setFreenoveDialogOpen(true);
  };

  const handleFreenoveDialogClose = () => {
    setFreenoveDialogOpen(false);
  };

  const handleMicrobitDialogOpen = () => {
    setMicrobitDialogOpen(true);
  };

  const handleMicrobitDialogClose = () => {
    setMicrobitDialogOpen(false);
  };

  const genomeActions = () => {
    navigate("/genome/mode");
  };

  const showFreenoveDialog = () => {
    return (
      <>
        <Dialog open={freenoveDialogOpen} onClose={handleFreenoveDialogClose}>
          <DialogTitle>Freenove SmartCar Setup</DialogTitle>
          <DialogContent>
            <DialogContentText>
              4WD Smart Car is a RaspberryPi powered STEM kit distributed by <a href="https://freenove.com"> Freenove</a>. FEAGI has support for the
              controller board installed on this robot and can enable you to read sensory data from ultrasonic and
              infrared sensors while being able to control the motors and servos available on the smart-car through
              FEAGI and as a result of neuronal activities.<br/>
              <br/>
              If you are in possession of this robot, prior to proceeding to genome actions step, ensure you follow instructions located under
              <a href="https://github.com/feagi/feagi/wiki"> Freenove Setup Guide</a>
              to connect FEAGI with your robot.

            </DialogContentText>
          </DialogContent>
          <DialogActions>
            <Button autoFocus onClick={genomeActions}>
              Proceed to genome actions...
            </Button>
        </DialogActions>
        </Dialog>
      </>
    );
  };

  const showMicrobitDialog = () => {
    return (
      <>
        <Dialog open={microbitDialogOpen} onClose={handleMicrobitDialogClose}>
          <DialogTitle>Microbit Setup</DialogTitle>
          <DialogContent>
            <DialogContentText>
              The <a href="https://microbit.org"> Microbit</a> a pocket-sized computer that makes learning coding easy and fun.
              Microbit is packed with many capabilities but at this time FEAGI is only capable of interacting with its motor driver.

              <br/>
              <br/>
              If you are in possession of this robot, prior to proceeding to genome actions step, ensure you follow instructions located under
              <a href="https://github.com/feagi/feagi/wiki"> Microbit Setup Guide</a>
              to connect FEAGI with your robot.


            </DialogContentText>
            <DialogActions>
            <Button autoFocus onClick={genomeActions}>
              Proceed to genome actions...
            </Button>
        </DialogActions>
          </DialogContent>
        </Dialog>
      </>
    );
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select Robot and Follow Setup Instructions
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
              image={<GrRaspberry size={150} />}
              label="Freenove SmartCar"
              onClick={handleFreenoveDialogOpen}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<SiMicrobit size={150} />}
              label="BBC micro:bit"
              onClick={handleMicrobitDialogOpen}
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
      {showFreenoveDialog()}
      {showMicrobitDialog()}
    </>
  );
};

export default PhysicalRobots;
