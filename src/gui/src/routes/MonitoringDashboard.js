import React, { useState, useEffect } from "react";
// import CircularProgress from "@mui/material/CircularProgress";
import Stack from "@mui/material/Stack";
import Iframe from "react-iframe";
import Box from "@mui/material/Box";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import MenuIcon from "@mui/icons-material/Menu";
import Typography from "@mui/material/Typography";
import Button from "@mui/material/Button";
import Drawer from "@mui/material/Drawer";
import Divider from "@mui/material/Divider";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemIcon from "@mui/material/ListItemIcon";
import ReplayIcon from "@mui/icons-material/Replay";
import ListItemText from "@mui/material/ListItemText";
import Tooltip from "@mui/material/Tooltip";
import { Route, Routes, Navigate, useNavigate } from "react-router-dom";
import { MdAutoGraph } from 'react-icons/md';
import { BiDna } from 'react-icons/bi';
import { FiZap } from 'react-icons/fi';
import { GrRobot } from 'react-icons/gr';
import { FaRegMap } from 'react-icons/fa';
import { GiBrainDump } from 'react-icons/gi';
import FormControl from "@mui/material/FormControl";
import InputLabel from "@mui/material/InputLabel";
import Select from "@mui/material/Select";
import MenuItem from "@mui/material/MenuItem";
import ListItemButton from '@mui/material/ListItemButton';
import Checkbox from '@mui/material/Checkbox';
import CommentIcon from '@mui/icons-material/Comment';
import FormHelperText from "@mui/material/FormHelperText";

import ListSubheader from '@mui/material/ListSubheader';
import Switch from '@mui/material/Switch';
import Menu from '@mui/material/Menu';
import FeagiAPI from "../services/FeagiAPI";
import Dialog from "@mui/material/Dialog";
import DialogTitle from "@mui/material/DialogTitle";
import DialogContent from "@mui/material/DialogContent";
import DialogContentText from "@mui/material/DialogContentText";
import DialogActions from "@mui/material/DialogActions";


const MonitoringDashboard = (props) => {
  const [frameHeight, setFrameHeight] = useState("");
  const [godotFrameLoaded, setGodotFrameLoaded] = useState(false);
  const [gazeboFrameLoaded, setGazeboFrameLoaded] = useState(false);
  const [drawerOpen, setDrawerOpen] = useState(false);
  let navigate = useNavigate();

  useEffect(() => {
    const clientHeight = document.documentElement.scrollHeight;
    setTimeout(() => {
      setFrameHeight(clientHeight + "px");
    }, 100);

    // if (!window.sessionStorage.getItem("visited")) {
    //   setTimeout(() => {
    //     setGodotFrameLoaded(true);
    //   }, 10000);
    // } else {
    //   setGodotFrameLoaded(true);
    // }
    // window.sessionStorage.setItem("visited", true);
  }, []);

  const scrollHeightScaled =
    Math.round(document.documentElement.scrollHeight / 1.3) + "px";

  const handleGazeboLoad = () => {
    setGazeboFrameLoaded(true);
  };

  const handleMenuClick = () => {
    setDrawerOpen(!drawerOpen);
  };

  const handleGenomeReload = () => {
    FeagiAPI.resetGenome({
    });
    navigate("/genome/mode");
  };

  const handleActicityMonitor = () => {
    window.open("http://localhost:6082/d/Se3OI7f7k/feagi-brain-activity-analyzer?orgId=1&refresh=1s", "_blank", "noopener,noreferrer");
  };


  const defaultShocks = {};
    Object.keys(props.defaultShockOptions).forEach((key) => {
      // defaultShocks[props.defaultCorticalGenes[key][0]] =
      //   props.defaultCorticalGenes[key][1];
    });


  const [checked, setChecked] = React.useState(['none']);

  const handleToggle = (value) => () => {

    const currentIndex = checked.indexOf(value);
    const newChecked = [...checked];

    if (currentIndex === -1) {
      newChecked.push(value);
      FeagiAPI.postShockRobot({
        shock: [
          value
        ]
       });

    } else {
      newChecked.splice(currentIndex, 1);
      FeagiAPI.postShockRobot({
        shock: [
        ]
      });
    }
    setChecked(newChecked);

  };


  const drawerList = () => (
    <Box
      sx={{ width: "400px" }}
      role="presentation"
      onClick={() => setDrawerOpen(false)}
      onKeyDown={() => setDrawerOpen(false)}
    >
      <List>
        {/* <ListItem button>
          <ListItemIcon>
            <AccessAlarmIcon />
          </ListItemIcon>
          <ListItemText primary="Change Burst Frequency" />
        </ListItem>
        <Divider />
        <ListItem button>
          <ListItemIcon>
            <CameraAltIcon />
          </ListItemIcon>
          <ListItemText primary="Take Connectome Snapshot" />
        </ListItem>
        <Divider /> */}
        <ListItem button onClick={handleGenomeReload}>
          <ListItemIcon>
            <ReplayIcon />
          </ListItemIcon>
          <ListItemText primary="Reload Genome" />
        </ListItem>
        <Divider />
      </List>
    </Box>
  );

  const [anchorEl, setAnchorEl] = React.useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const [robotSelectorDialogOpen, setRobotSelectorDialogOpen] = useState(false);
  const [environmentSelectorDialogOpen, setEnvironmentSelectorDialogOpen] = useState(false);


  const handleRobotSelectorDialogOpen = () => {
    setRobotSelectorDialogOpen(true);
  };

  const handleRobotSelectorDialogClose = () => {
    setRobotSelectorDialogOpen(false);
  };

  const handleEnvironmentSelectorDialogOpen = () => {
    setEnvironmentSelectorDialogOpen(true);
  };

  const handleEnvironmentSelectorDialogClose = () => {
    setEnvironmentSelectorDialogOpen(false);
  };

  const showRobotSelectorDialog = () => {
    return (
      <>
        <Dialog open={robotSelectorDialogOpen} onClose={handleRobotSelectorDialogClose}>
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
        </Dialog>
      </>
    );
  };

  const showEnvironmentSelectorDialog = () => {
    return (
      <>
        <Dialog open={environmentSelectorDialogOpen} onClose={handleEnvironmentSelectorDialogClose}>
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
        </Dialog>
      </>
    );
  };


  return (
    <>
    <Stack sx={{ flexGrow: 1 }}>
      <AppBar
          position="static"
          color="transparent"
          sx={{ mt: 1, mb: 0, ml: 1, mr: 1 }}
      >
          <Toolbar>
            <Tooltip title="Reload Genome" placement="top">
            <IconButton
              color="inherit"
              onClick={handleGenomeReload}
              // edge="start"
            >
              <BiDna />
            </IconButton>
            </Tooltip>

            <Tooltip title="Launch Brain Activity Monitor" placement="top">
                <IconButton
                  color="inherit"
                  onClick={handleActicityMonitor}
                  // edge="start"
                >
                  <MdAutoGraph />
                </IconButton>
            </Tooltip>

            <Tooltip title="Environment Selector" placement="top">
                <IconButton
                  color="inherit"
                  onClick={handleEnvironmentSelectorDialogOpen}
                  // edge="start"
                >
                  <FaRegMap />
                </IconButton>
            </Tooltip>

            <Tooltip title="Robot Selector" placement="top">
                <IconButton
                  color="inherit"
                  onClick={handleRobotSelectorDialogOpen}
                  // edge="start"
                >
                  <GrRobot />
                </IconButton>
            </Tooltip>


            <Tooltip title="Shock Admin" placement="top">
              <div>
                <IconButton
                  id="basic-button"
                  aria-controls={open ? 'basic-menu' : undefined}
                  aria-haspopup="true"
                  aria-expanded={open ? 'true' : undefined}
                  onClick={handleClick}
                >
                   <FiZap />
                  {/*Shock Admin*/}
                </IconButton>
                <Menu
                  id="basic-menu"
                  anchorEl={anchorEl}
                  open={open}
                  onClose={handleClose}
                  MenuListProps={{
                    'aria-labelledby': 'basic-button',
                  }}
                >
                  <List
                    sx={{ width: '100%', maxWidth: 360, bgcolor: 'background.paper' }}
                    subheader={<ListSubheader>Shock Options</ListSubheader>}
                    >
                    <ListItem>
                      <ListItemText id="switch-list-label-shock_scenario_1" primary="Dark surface shock" />
                      <Switch
                        edge="end"
                        onChange={handleToggle('shock_scenario_1')}
                        checked={checked.indexOf('shock_scenario_1') !== -1}
                        inputProps={{
                          'aria-labelledby': 'switch-list-label-shock_scenario_1',
                        }}
                      />
                    </ListItem>
                    <ListItem>

                      <ListItemText id="switch-list-label-shock_scenario_2" primary="Bright surface shock" />
                      <Switch
                        edge="end"
                        onChange={handleToggle('shock_scenario_2')}
                        checked={checked.indexOf('shock_scenario_2') !== -1}
                        inputProps={{
                          'aria-labelledby': 'switch-list-label-shock_scenario_2',
                        }}
                      />
                    </ListItem>
                  </List>
                </Menu>
              </div>
            </Tooltip>


            {/*<Tooltip title="Preserve Brain State (Coming Soon..)">*/}
            {/*    <IconButton*/}
            {/*      color="inherit"*/}
            {/*      // onClick={handleActicityMonitor}*/}
            {/*      // edge="start"*/}
            {/*    >*/}
            {/*      <GiBrainDump />*/}
            {/*    </IconButton>*/}
            {/*</Tooltip>*/}

          </Toolbar>
        </AppBar>
    </Stack>

    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={1}
      sx={{ mt: 1, mb: 2, ml: 1, mr: 1 }}
    >
      {/* {godotFrameLoaded ? ( */}

      <Drawer
        anchor="left"
        variant="temporary"
        elevation={3}
        open={drawerOpen}
        onClose={() => setDrawerOpen(false)}
      >
        <Box textAlign="center" role="presentation">
          <Typography variant="h5" component="div" sx={{ m: 2 }}>
            Configuration
          </Typography>
          <Divider />
        </Box>
        {drawerList()}
      </Drawer>
      <iframe
        id="godotFrame"
        src="http://localhost:6081"
        width="50%"
        height={scrollHeightScaled}
      />
      {/* ) : (
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            height: scrollHeightScaled,
            width: "50%",
          }}
        >
          <CircularProgress size="150px" />
        </div>
      )} */}
      <Iframe
        className="iframe"
        id="gazeboFrame"
        url="http://localhost:6080"
        width="50%"
        height={scrollHeightScaled}
        onLoad={handleGazeboLoad}
      />
    </Stack>
    {showRobotSelectorDialog()}
    {showEnvironmentSelectorDialog()}
    </>

  );
};

export default MonitoringDashboard;
