import React, { useState } from "react";
import { Route, Routes, Navigate } from "react-router-dom";
import Environment from "./routes/Environment";
import GenomeAssembler from "./routes/GenomeAssembler";
import GenomeMode from "./routes/GenomeMode";
import Mapping from "./routes/Mapping";
import Motor from "./routes/Motor";
import Sensory from "./routes/Sensory";
import MonitoringDashboard from "./routes/MonitoringDashboard";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  const [definedMotor, setDefinedMotor] = useState([]);
  const [definedSensory, setDefinedSensory] = useState([]);
  const [definedMappings, setDefinedMappings] = useState([]);

  return (
    <>
      <ResponsiveAppBar />
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
            />
          }
        />
        <Route
          path="/brain/motor"
          element={
            <Motor
              definedMotor={definedMotor}
              setDefinedMotor={setDefinedMotor}
            />
          }
        />
        <Route
          path="/brain/sensory"
          element={
            <Sensory
              definedSensory={definedSensory}
              setDefinedSensory={setDefinedSensory}
            />
          }
        />
        <Route path="/monitoring" element={<MonitoringDashboard />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
