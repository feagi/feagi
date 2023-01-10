import React, { useState } from "react";
import { GrRaspberry } from "react-icons/gr";
import { SiMicrobit } from "react-icons/si";
import { MdOutlinePsychology } from "react-icons/md";
import {HiOutlineVideoCamera} from "react-icons/hi"
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
import {Img} from "react-image";


const PhysicalRobots = () => {
  const [freenoveDialogOpen, setFreenoveDialogOpen] = useState(false);
  const [microbitDialogOpen, setMicrobitDialogOpen] = useState(false);
  const [psychopyDialogOpen, setPsychopyDialogOpen] = useState(false);
  const [webcamDialogOpen, setWebcamDialogOpen] = useState(false);
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

  const handlePsychopyDialogOpen = () => {
    setPsychopyDialogOpen(true);
  };

  const handlePsychopyDialogClose = () => {
    setPsychopyDialogOpen(false);
  };

  const handleWebcamDialogOpen = () => {
    setWebcamDialogOpen(true);
  };

  const handleWebcamDialogClose = () => {
    setWebcamDialogOpen(false);
  };

  const genomeActions = () => {
    navigate("/monitoring-go");
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
              Start monitoring...
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
              Start monitoring...
            </Button>
        </DialogActions>
          </DialogContent>
        </Dialog>
      </>
    );
  };

  const showPsychopyDialog = () => {
    return (
      <>
        <Dialog open={psychopyDialogOpen} onClose={handlePsychopyDialogClose}>
          <DialogTitle>Psychopy Setup</DialogTitle>
          <DialogContent>
            <DialogContentText>
              TBD



            </DialogContentText>
            <DialogActions>
            <Button autoFocus onClick={genomeActions}>
              Start monitoring...
            </Button>
        </DialogActions>
          </DialogContent>
        </Dialog>
      </>
    );
  };


  const showWebcamDialog = () => {
    return (
      <>
        <Dialog open={webcamDialogOpen} onClose={handleWebcamDialogClose}>
          <DialogTitle>Webcam Setup</DialogTitle>
          <DialogContent>
            <DialogContentText>
              TBD
            </DialogContentText>
            <DialogActions>
            <Button autoFocus onClick={genomeActions}>
              Start monitoring...
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
        Choose Your Embodiment
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
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              // image={<Img src={require('../assets/Psychopy.png')} width="150" height="150" />}

                image={<MdOutlinePsychology size={150} />}
              label="Psychopy"
              onClick={handlePsychopyDialogOpen}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="robot-card">
            <MenuCard
              image={<HiOutlineVideoCamera size={150} />}
              label="Webcam"
              onClick={handleWebcamDialogOpen}
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
      {showFreenoveDialog()}
      {showMicrobitDialog()}
      {showPsychopyDialog()}
      {showWebcamDialog()}
    </>
  );
};

export default PhysicalRobots;
