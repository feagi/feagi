import React, { useState } from "react";
import Motor from "../components/Motor";

const DefineMotor = () => {
  const [definedMotor, setDefinedMotor] = useState([]);
  return (
    <div>
      <Motor setDefinedMotor={setDefinedMotor} />
    </div>
  );
};

export default DefineMotor;
