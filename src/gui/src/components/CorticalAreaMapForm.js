import React, { useState, useEffect } from "react";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";
import Fab from "@mui/material/Fab";
import FormControl from "@mui/material/FormControl";
import InputLabel from "@mui/material/InputLabel";
import MenuItem from "@mui/material/MenuItem";
import Select from "@mui/material/Select";
import TextField from "@mui/material/TextField";
import Typography from "@mui/material/Typography";
import FeagiAPI from "../services/FeagiAPI";

const CorticalAreaMapForm = (props) => {
  const [destinationCount, setDestinationCount] = useState(1);
  const [sensoryAreas, setSensoryAreas] = useState([]);
  const [motorAreas, setMotorAreas] = useState([]);
  const [predefinedSynapseRules, setPredefinedSynapseRules] = useState({});

  const [selectedCorticalArea, setSelectedCorticalArea] = useState("");
  const [selectedRule, setSelectedRule] = useState("");

  /* 

    - state might need to be an object with keys pertaining to the mapping #
      for the respective cortical area

    - use idx value from main .map to access unique keys in state object and
      update their values
  
  */

  useEffect(() => {
    FeagiAPI.getBaselineMotor().then((items) => setMotorAreas(items));
    FeagiAPI.getBaselineSensory().then((items) => setSensoryAreas(items));
    FeagiAPI.getBaselineMorphology().then((items) =>
      setPredefinedSynapseRules(items)
    );
  }, []);

  const handleAreaChange = (event) => {
    setSelectedCorticalArea(event.target.value);
  };

  const handleRuleChange = (event) => {
    setSelectedRule(event.target.value);
  };

  const handleAdd = () => {
    setDestinationCount(destinationCount + 1);
  };

  const handleRemove = () => {
    if (destinationCount > 0) {
      setDestinationCount(destinationCount - 1);
    } else {
      setDestinationCount(0);
    }
  };

  return (
    <>
      <Typography gutterBottom variant="h5" component="div" sx={{ mb: 3 }}>
        Cortical Mapping for {props.corticalArea}
      </Typography>
      {Array.from(Array(destinationCount)).map((item, idx) => {
        return (
          <div key={`${item}-${idx}`}>
            <Typography variant="h6" component="div" sx={{ mt: 2 }}>
              Destination {idx + 1}
            </Typography>
            <FormControl sx={{ m: 2, minWidth: 120 }}>
              <InputLabel id={`dest-select-label-${idx}`}>
                Cortical Area
              </InputLabel>
              <Select
                labelId={`dest-select-label-${idx}`}
                id={`dest-select-${idx}`}
                value={selectedCorticalArea}
                label="Cortical Area"
                onChange={handleAreaChange}
                sx={{ width: "250px" }}
              >
                {sensoryAreas.concat(motorAreas).map((area) => {
                  return (
                    <MenuItem key={area} value={area}>
                      {area}
                    </MenuItem>
                  );
                })}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 2, minWidth: 120 }}>
              <InputLabel id={`rule-select-label-${idx}`}>
                Pre-defined Rule
              </InputLabel>
              <Select
                labelId={`rule-select-label-${idx}`}
                id={`rule-select-${idx}`}
                value={selectedRule}
                label="Pre-defined Rule"
                onChange={handleRuleChange}
                sx={{ width: "250px" }}
              >
                {Object.keys(predefinedSynapseRules).map((rule) => {
                  return (
                    <MenuItem key={rule} value={rule}>
                      {rule}
                    </MenuItem>
                  );
                })}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 2, minWidth: 200 }}>
              <TextField
                id="rule-def-field"
                label="Rule Definition"
                variant="outlined"
                sx={{ width: "250px" }}
              >
                {console.log(typeof predefinedSynapseRules[selectedRule])}
              </TextField>
            </FormControl>
          </div>
        );
      })}
      <div>
        <Fab size="small" color="primary" aria-label="add" sx={{ m: 1 }}>
          <AddIcon onClick={handleAdd} />
        </Fab>
        <Fab size="small" color="primary" aria-label="add" sx={{ m: 1 }}>
          <RemoveIcon onClick={handleRemove} />
        </Fab>
      </div>
    </>
  );
};

export default CorticalAreaMapForm;
