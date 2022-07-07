import React, { lazy, Suspense, useState, useEffect } from "react";
import { Route, Routes, Navigate, useNavigate } from "react-router-dom";
// import AccessAlarmIcon from "@mui/icons-material/AccessAlarm";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button";
import CircularProgress from "@mui/material/CircularProgress";
// import CameraAltIcon from "@mui/icons-material/CameraAlt";
import Divider from "@mui/material/Divider";
import Drawer from "@mui/material/Drawer";
import IconButton from "@mui/material/IconButton";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemIcon from "@mui/material/ListItemIcon";
import ListItemText from "@mui/material/ListItemText";
import MenuIcon from "@mui/icons-material/Menu";
import ReplayIcon from "@mui/icons-material/Replay";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import CssBaseline from "@mui/material/CssBaseline";
import { ThemeProvider, createTheme } from "@mui/material/styles";
// import ProgressStepper from "./components/ProgressStepper";
import FeagiAPI from "./services/FeagiAPI";

const Environment = lazy(() => import("./routes/Environment"));
const GenomeAssembler = lazy(() => import("./routes/GenomeAssembler"));
const GenomeMode = lazy(() => import("./routes/GenomeMode"));
const CorticalAreaEditor = lazy(() => import("./routes/CorticalAreaEditor"));
const Mapping = lazy(() => import("./routes/Mapping"));
const Sensorimotor = lazy(() => import("./routes/Sensorimotor"));
const MonitoringDashboard = lazy(() => import("./routes/MonitoringDashboard"));
const PhysicalRobots = lazy(() => import("./routes/PhysicalRobots"));
const DefaultGenomes = lazy(() => import("./routes/DefaultGenomes"));

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
  const [defaultGenomeData, setDefaultGenomeData] = useState({});
  const [drawerOpen, setDrawerOpen] = useState(false);
  let navigate = useNavigate();

  useEffect(() => {
    const currentDomain = window.location.href;
    if (currentDomain.includes("127.0.0.1:3000")) {
      const redirectDomain = currentDomain.replace(
        "127.0.0.1:3000",
        "localhost:3000"
      );
      window.location.replace(redirectDomain);
    }

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

    FeagiAPI.getDefaultGenomes().then((response) => {
      setDefaultGenomeData(response["genomes"]);
    });
  }, []);

  const appTheme = createTheme({
    palette: {
      background: {
        default: "#ebe8e8",
      },
    },
  });

  const handleApiDocsClick = () => {
    window.open("http://localhost:8000/docs", "_blank", "noopener,noreferrer");
  };

  const handleHelpClick = () => {
    window.open(
      "https://github.com/feagi/feagi/wiki#platform-overview",
      "_blank",
      "noopener,noreferrer"
    );
  };


  return (
    <>
      <ThemeProvider theme={appTheme}>
        <CssBaseline />
        <Box sx={{ flexGrow: 1 }}>
          <AppBar position="static">
            <Toolbar>
              {/*<IconButton*/}
              {/*  color="inherit"*/}
              {/*  onClick={handleMenuClick}*/}
              {/*  edge="start"*/}
              {/*>*/}
              {/*  <MenuIcon />*/}
              {/*</IconButton>*/}
              <Typography variant="h6" sx={{ ml: 4, flexGrow: 1 }}>
                Framework for Evolutionary Artificial General Intelligence
                (FEAGI)
              </Typography>
              <div
                style={{
                  display: "flex",
                  justifyContent: "right",
                  alignItems: "right",
                }}
              >
                <Button
                  variant="text"
                  color="inherit"
                  onClick={handleApiDocsClick}
                >
                  API Docs
                </Button>
                <Button
                  variant="text"
                  color="inherit"
                  onClick={handleHelpClick}
                >
                  Help
                </Button>
              </div>
            </Toolbar>
          </AppBar>
        </Box>
        <Suspense
          fallback={
            <div
              style={{
                position: "fixed",
                top: "50%",
                left: "50%",
                transform: "translate(-50%, -50%)",
              }}
            >
              <CircularProgress size="150px" />
            </div>
          }
        >
          <Routes>
            <Route path="/" element={<Navigate replace to="/environment" />} />
            <Route path="/environment" element={<Environment />} />
            <Route path="/robot/physical" element={<PhysicalRobots />} />
            <Route path="/genome/mode" element={<GenomeMode />} />
            <Route
              path="/genome/defaults"
              element={<DefaultGenomes defaultGenomeData={defaultGenomeData} />}
            />
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
        </Suspense>
      </ThemeProvider>
    </>
  );
}

export default App;
