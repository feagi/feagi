import React, { useState, useEffect } from "react";
import { Route, Routes, Navigate } from "react-router-dom";
import AccessAlarmIcon from "@mui/icons-material/AccessAlarm";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import CameraAltIcon from "@mui/icons-material/CameraAlt";
import Container from "@mui/material/Container";
import Divider from "@mui/material/Divider";
import Drawer from "@mui/material/Drawer";
import IconButton from "@mui/material/IconButton";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemIcon from "@mui/material/ListItemIcon";
import ListItemText from "@mui/material/ListItemText";
import MenuIcon from "@mui/icons-material/Menu";
import Stack from "@mui/material/Stack";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import CssBaseline from "@mui/material/CssBaseline";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import Environment from "./routes/Environment";
import GenomeAssembler from "./routes/GenomeAssembler";
import GenomeMode from "./routes/GenomeMode";
import CorticalAreaEditor from "./routes/CorticalAreaEditor";
import Mapping from "./routes/Mapping";
import Sensorimotor from "./routes/Sensorimotor";
import MonitoringDashboard from "./routes/MonitoringDashboard";
import ProgressStepper from "./components/ProgressStepper";
import FeagiAPI from "./services/FeagiAPI";

function App() {
  const [definedMotor, setDefinedMotor] = useState([]);
  const [selectedMotor, setSelectedMotor] = useState([]);
  const [definedSensory, setDefinedSensory] = useState([]);
  const [selectedSensory, setSelectedSensory] = useState([]);
  const [definedMappings, setDefinedMappings] = useState([]);
  const [customAreas, setCustomAreas] = useState([]);
  const [definedAreas, setDefinedAreas] = useState([]);
  const [defaultMorphologyScalarX, setDefaultMorphologyScalarX] =
    useState(null);
  const [defaultMorphologyScalarY, setDefaultMorphologyScalarY] =
    useState(null);
  const [defaultMorphologyScalarZ, setDefaultMorphologyScalarZ] =
    useState(null);
  const [defaultPscMultiplier, setDefaultPscMultiplier] = useState(null);
  const [defaultPlasticityFlag, setDefaultPlasticityFlag] = useState(null);
  const [defaultSynapseRules, setDefaultSynapseRules] = useState(null);
  const [defaultCorticalGenes, setDefaultCorticalGenes] = useState({});
  const [drawerOpen, setDrawerOpen] = useState(false);

  const currentDomain = window.location.href;
  if (currentDomain.includes("127.0.0.1:3000")) {
    const redirectDomain = currentDomain.replace(
      "127.0.0.1:3000",
      "localhost:3000"
    );
    window.location.replace(redirectDomain);
  }

  useEffect(() => {
    FeagiAPI.getBaselineCorticalGenes().then((items) =>
      setDefaultCorticalGenes(items)
    );

    FeagiAPI.getBaselineMorphology().then((rules) =>
      setDefaultSynapseRules(rules)
    );

    let defaultMorphologySetterArray = [
      setDefaultMorphologyScalarX,
      setDefaultMorphologyScalarY,
      setDefaultMorphologyScalarZ,
    ];
    FeagiAPI.getBaselineMorphologyScalar().then((items) =>
      items.forEach((item, index) => defaultMorphologySetterArray[index](item))
    );

    FeagiAPI.getBaselinePscMultiplier().then((multiplier) =>
      setDefaultPscMultiplier(multiplier)
    );

    FeagiAPI.getBaselinePlasticityFlag().then((flag) =>
      setDefaultPlasticityFlag(flag)
    );
  }, []);

  const appTheme = createTheme({
    palette: {
      background: {
        default: "#ebe8e8",
      },
    },
  });

  const handleMenuClick = () => {
    setDrawerOpen(!drawerOpen);
  };

  const drawerList = () => (
    <Box
      sx={{ width: "400px" }}
      role="presentation"
      onClick={() => setDrawerOpen(false)}
      onKeyDown={() => setDrawerOpen(false)}
    >
      <List>
        {["Change Burst Frequency", "Take Connectome Snapshot"].map(
          (text, index) => (
            <ListItem button key={text}>
              <ListItemIcon>
                {index % 2 === 0 ? <AccessAlarmIcon /> : <CameraAltIcon />}
              </ListItemIcon>
              <ListItemText primary={text} />
            </ListItem>
          )
        )}
      </List>
      <Divider />
    </Box>
  );

  return (
    <>
      <ThemeProvider theme={appTheme}>
        <CssBaseline />
        <AppBar position="static">
          <Container maxWidth="xxl">
            <Stack
              direction="row"
              alignItems="center"
              justifyContent="left"
              spacing={6}
              sx={{ m: 1 }}
            >
              <IconButton
                color="inherit"
                onClick={handleMenuClick}
                edge="start"
              >
                <MenuIcon />
              </IconButton>
              <Toolbar disableGutters>
                <Typography variant="h6">
                  Framework for Evolutionary Artificial General Intelligence
                  (FEAGI)
                </Typography>
              </Toolbar>
            </Stack>
          </Container>
        </AppBar>
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
        <Routes>
          <Route path="/" element={<Navigate replace to="/environment" />} />
          <Route path="/environment" element={<Environment />} />
          <Route path="/genome/mode" element={<GenomeMode />} />
          <Route
            path="/genome/assemble"
            element={
              <GenomeAssembler
                definedMotor={definedMotor}
                definedSensory={definedSensory}
                definedMappings={definedMappings}
                definedAreas={definedAreas}
              />
            }
          />
          <Route
            path="/brain/mapping"
            element={
              <Mapping
                definedMappings={definedMappings}
                setDefinedMappings={setDefinedMappings}
                definedSensory={definedSensory}
                definedMotor={definedMotor}
                definedAreas={definedAreas}
                setDefinedAreas={setDefinedAreas}
                defaultMorphologyScalarX={defaultMorphologyScalarX}
                defaultMorphologyScalarY={defaultMorphologyScalarY}
                defaultMorphologyScalarZ={defaultMorphologyScalarZ}
                defaultPscMultiplier={defaultPscMultiplier}
                defaultPlasticityFlag={defaultPlasticityFlag}
                defaultSynapseRules={defaultSynapseRules}
              />
            }
          />
          <Route
            path="/brain/editor"
            element={
              <CorticalAreaEditor
                selectedSensory={selectedSensory}
                setSelectedSensory={setSelectedSensory}
                definedSensory={definedSensory}
                setDefinedSensory={setDefinedSensory}
                selectedMotor={selectedMotor}
                definedMotor={definedMotor}
                setDefinedMotor={setDefinedMotor}
                customAreas={customAreas}
                setCustomAreas={setCustomAreas}
                definedAreas={definedAreas}
                setDefinedAreas={setDefinedAreas}
                defaultCorticalGenes={defaultCorticalGenes}
              />
            }
          />
          <Route
            path="/brain/sensorimotor"
            element={
              <Sensorimotor
                selectedSensory={selectedSensory}
                setSelectedSensory={setSelectedSensory}
                definedSensory={definedSensory}
                setDefinedSensory={setDefinedSensory}
                selectedMotor={selectedMotor}
                setSelectedMotor={setSelectedMotor}
                definedMotor={definedMotor}
                setDefinedMotor={setDefinedMotor}
                definedAreas={definedAreas}
                setDefinedAreas={setDefinedAreas}
                defaultCorticalGenes={defaultCorticalGenes}
              />
            }
          />
          <Route path="/monitoring" element={<MonitoringDashboard />} />
        </Routes>
        {/* <div
          style={{
            position: "absolute",
            left: "50px",
            bottom: "50px",
            right: "50px",
          }}
        >
          <ProgressStepper />
        </div> */}
      </ThemeProvider>
    </>
  );
}

export default App;
