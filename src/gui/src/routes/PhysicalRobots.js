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

const PhysicalRobots = () => {
  const [freenoveDialogOpen, setFreenoveDialogOpen] = useState(false);
  const [microbitDialogOpen, setMicrobitDialogOpen] = useState(false);

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

  const showFreenoveDialog = () => {
    return (
      <>
        <Dialog open={freenoveDialogOpen} onClose={handleFreenoveDialogClose}>
          <DialogTitle>Freenove SmartCar Setup</DialogTitle>
          <DialogContent>
            <DialogContentText>
              Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
              eiusmod tempor incididunt ut labore et dolore magna aliqua.
              Pretium vulputate sapien nec sagittis aliquam malesuada bibendum.
              Id ornare arcu odio ut sem nulla pharetra diam sit. Eget sit amet
              tellus cras adipiscing enim. Vel risus commodo viverra maecenas
              accumsan lacus.
            </DialogContentText>
          </DialogContent>
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
              Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do
              eiusmod tempor incididunt ut labore et dolore magna aliqua. Nunc
              vel risus commodo viverra maecenas accumsan lacus vel. Aliquet
              sagittis id consectetur purus ut faucibus pulvinar elementum.
              Tincidunt dui ut ornare lectus sit amet est placerat. Ullamcorper
              sit amet risus nullam eget.
            </DialogContentText>
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
