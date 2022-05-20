import React, { useState, useEffect } from "react";
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
import FeagiAPI from "./services/FeagiAPI";

function App() {
  const [definedMotor, setDefinedMotor] = useState([]);
  const [definedSensory, setDefinedSensory] = useState([]);
  const [definedMappings, setDefinedMappings] = useState([]);

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
          path="/brain/motor"
          element={
            <Motor
              definedMotor={definedMotor}
              setDefinedMotor={setDefinedMotor}
              defaultCorticalGenes={defaultCorticalGenes}
            />
          }
        />
        <Route
          path="/brain/sensory"
          element={
            <Sensory
              definedSensory={definedSensory}
              setDefinedSensory={setDefinedSensory}
              defaultCorticalGenes={defaultCorticalGenes}
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
