import React, { useState } from "react";
import Button from "@mui/material/Button";

function Form() {
  const [step, setStep] = useState(0);

  return (
    <div className="form">
      <div className="progress-bar">
        <div className="form-container">
          <div className="header"></div>
          <div className="body"></div>
          <div className="footer"></div>
          <Button
            variant="contained"
            onClick={() => setStep((currentStep) => currentStep + 1)}
          >
            NEXT
          </Button>
          <Button variant="contained">PREV</Button>
        </div>
      </div>
    </div>
  );
}

export default Form;
