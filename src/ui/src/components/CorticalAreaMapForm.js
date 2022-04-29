import React, { useState } from "react";
import AddIcon from "@mui/icons-material/Add";
import RemoveIcon from "@mui/icons-material/Remove";
import Fab from "@mui/material/Fab";
import FormControl from "@mui/material/FormControl";
import InputLabel from "@mui/material/InputLabel";
import MenuItem from "@mui/material/MenuItem";
import Select from "@mui/material/Select";
import Typography from "@mui/material/Typography";

const CorticalAreaMapForm = (props) => {
  const [destinationCount, setDestinationCount] = useState(1);

  const handleChange = (event) => {
    console.log(event);
    // setSomeValue(event.target.value);
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
                value=""
                label="Cortical Area"
                onChange={handleChange}
                sx={{ width: "250px" }}
              >
                <MenuItem value={100}>TEST</MenuItem>
              </Select>
            </FormControl>
            <FormControl sx={{ m: 2, minWidth: 120 }}>
              <InputLabel id={`rule-select-label-${idx}`}>Rule Type</InputLabel>
              <Select
                labelId={`rule-select-label-${idx}`}
                id={`rule-select-${idx}`}
                value=""
                label="Synaptogenesis Rule"
                onChange={handleChange}
                sx={{ width: "250px" }}
              >
                <MenuItem value={100}>TEST</MenuItem>
              </Select>
            </FormControl>
            <FormControl sx={{ m: 2, minWidth: 120 }}>
              <InputLabel id={`rule-type-select-label-${idx}`}>Rule</InputLabel>
              <Select
                labelId={`rule-type-select-label-${idx}`}
                id={`rule-type-select-${idx}`}
                value=""
                label="Synaptogenesis Rule"
                onChange={handleChange}
                sx={{ width: "250px" }}
              >
                <MenuItem value={100}>TEST</MenuItem>
              </Select>
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
