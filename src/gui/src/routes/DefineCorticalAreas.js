import React, { useState } from "react";
import ChooseMotor from "../components/ChooseMotor";
import ChooseSensory from "../components/ChooseSensory";
import CorticalAreaMapping from "../components/CorticalAreaMapping";

const DefineCorticalAreas = () => {
  const [definedSensory, setDefinedSensory] = useState([]);
  const [definedMotor, setDefinedMotor] = useState([]);

  return (
    <>
      <ChooseSensory setDefinedSensory={setDefinedSensory} />
      <ChooseMotor setDefinedMotor={setDefinedMotor} />
      <CorticalAreaMapping
        definedSensory={definedSensory}
        definedMotor={definedMotor}
      />
    </>
  );
};

export default DefineCorticalAreas;
