import React, { useState } from "react";
import AddIcon from "@mui/icons-material/Add";
import Checkbox from "@mui/material/Checkbox";
import DeleteIcon from "@mui/icons-material/Delete";
import Divider from "@mui/material/Divider";
import Fab from "@mui/material/Fab";
import FormControl from "@mui/material/FormControl";
import FormControlLabel from "@mui/material/FormControlLabel";
import FormHelperText from "@mui/material/FormHelperText";
import Grid from "@mui/material/Grid";
import IconButton from "@mui/material/IconButton";
import InputLabel from "@mui/material/InputLabel";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemText from "@mui/material/ListItemText";
import MenuItem from "@mui/material/MenuItem";
import SaveIcon from "@mui/icons-material/Save";
import Select from "@mui/material/Select";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";

const CorticalAreaMapForm = (props) => {
  const [selectedArea, setSelectedArea] = useState("");
  const [selectedRule, setSelectedRule] = useState("");
  const [mappedAreas, setMappedAreas] = useState([]);
  const [plasticity, setPlasticity] = useState(props.defaultPlasticityFlag);
  const [pscMultiplier, setPscMultiplier] = useState(
    props.defaultPcsMultiplier
  );
  const [morphologyScalarX, setMorphologyScalarX] = useState(
    props.defaultMorphologyScalarX
  );
  const [morphologyScalarY, setMorphologyScalarY] = useState(
    props.defaultMorphologyScalarY
  );
  const [morphologyScalarZ, setMorphologyScalarZ] = useState(
    props.defaultMorphologyScalarZ
  );

  const handleAreaChange = (event) => {
    setSelectedArea(event.target.value);
  };

  const handleRuleChange = (event) => {
    setSelectedRule(event.target.value);
  };

  const handleAdd = () => {
    setMappedAreas([
      ...mappedAreas,
      {
        dstArea: selectedArea,
        rule: selectedRule,
        info: props.defaultSynapseRules[selectedRule],
        morphologyScalar: [
          parseInt(morphologyScalarX),
          parseInt(morphologyScalarY),
          parseInt(morphologyScalarZ),
        ],
        pscMultiplier: pscMultiplier,
        plasticity: plasticity,
      },
    ]);
    setSelectedArea("");
    setSelectedRule("");
    setMorphologyScalarX(props.defaultMorphologyScalarX);
    setMorphologyScalarY(props.defaultMorphologyScalarY);
    setMorphologyScalarZ(props.defaultMorphologyScalarZ);
    setPscMultiplier(props.defaultPscMultiplier);
    setPlasticity(props.defaultPlasticityFlag);
  };

  const handleMappingDelete = (index) => {
    let updatedMappings = [...mappedAreas];
    updatedMappings.splice(index, 1);
    setMappedAreas(Array.from(updatedMappings));
    props.setDefinedMappings(Array.from(updatedMappings));
  };

  const handleSave = () => {
    props.setDefinedMappings({
      ...props.definedMappings,
      [props.srcCorticalArea.toLowerCase()]: mappedAreas,
    });
    props.setDialogOpen(false);
  };

  const handleCheckboxChange = (event) => {
    setPlasticity(event.target.checked);
  };

  const handlePscChange = (event) => {
    setPscMultiplier(event.target.value);
  };

  return (
    <>
      <Typography gutterBottom variant="h5" component="div" sx={{ mb: 4 }}>
        {props.srcCorticalArea} Area Cortical Mapping
      </Typography>
      <div>
        <Typography variant="h6" component="div" sx={{ mt: 2, mb: 1 }}>
          Select mapping destination and type
        </Typography>
        <Divider />
        <FormControl sx={{ mt: 2, mr: 1, mb: 2 }}>
          <InputLabel id="dest-select-label">Cortical Area</InputLabel>
          <Select
            labelId="dest-select-label"
            id="dest-select"
            value={selectedArea}
            label="Cortical Area"
            onChange={handleAreaChange}
            sx={{ width: "250px" }}
          >
            {props.availableMappingAreas.map((area) => {
              return (
                <MenuItem key={area} value={area}>
                  {area}
                </MenuItem>
              );
            })}
          </Select>
          <FormHelperText>Required</FormHelperText>
        </FormControl>
        <FormControl sx={{ mt: 2, mr: 1, mb: 2 }}>
          <InputLabel id="rule-select-label">Pre-defined Rule</InputLabel>
          <Select
            labelId="rule-select-label"
            id="rule-select"
            value={selectedRule}
            label="Pre-defined Rule"
            onChange={handleRuleChange}
            sx={{ width: "250px" }}
          >
            {Object.keys(props.defaultSynapseRules).map((rule) => {
              return (
                <MenuItem key={rule} value={rule}>
                  {rule}
                </MenuItem>
              );
            })}
          </Select>
          <FormHelperText>Required</FormHelperText>
        </FormControl>
        <FormControl sx={{ mt: 2, mr: 1, mb: 2 }}>
          <TextField
            disabled
            id="rule-def-field"
            label={
              selectedRule
                ? JSON.stringify(props.defaultSynapseRules[selectedRule])
                : "Rule info"
            }
            variant="outlined"
            sx={{ width: "250px" }}
          />
        </FormControl>
      </div>
      <div>
        <InputLabel sx={{ width: "150px", mt: 1 }}>
          Morphology Scalar
        </InputLabel>
        <Stack direction="row" alignItems="center" spacing={1} sx={{ mb: 2 }}>
          <TextField
            id="filled-basic-msx"
            defaultValue={1}
            helperText="X"
            variant="outlined"
            onChange={(e) => setMorphologyScalarX(e.target.value)}
            sx={{ width: "50px" }}
          />
          <TextField
            id="filled-basic-msy"
            defaultValue={1}
            helperText="Y"
            variant="outlined"
            onChange={(e) => setMorphologyScalarY(e.target.value)}
            sx={{ width: "50px" }}
          />
          <TextField
            id="filled-basic-msz"
            defaultValue={1}
            helperText="Z"
            variant="outlined"
            onChange={(e) => setMorphologyScalarZ(e.target.value)}
            sx={{ width: "50px" }}
          />
          <div>
            <Stack
              direction="row"
              alignItems="center"
              spacing={2}
              sx={{ ml: 3 }}
            >
              <TextField
                label="PSC Multiplier"
                type="number"
                defaultValue={1}
                helperText=" "
                onChange={(event) => handlePscChange(event)}
                sx={{ width: "110px" }}
              />
              <FormControl sx={{ width: "20px", mt: 1 }}>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={plasticity}
                      onChange={handleCheckboxChange}
                    />
                  }
                  label="Plasticity"
                />
              </FormControl>
            </Stack>
          </div>
        </Stack>
      </div>
      <div>
        <Tooltip
          title={
            !(selectedArea && selectedRule)
              ? "Select area/rule first..."
              : "Add mapping..."
          }
        >
          <span>
            <Fab
              size="small"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!(selectedArea && selectedRule)}
              onClick={handleAdd}
            >
              <AddIcon />
            </Fab>
          </span>
        </Tooltip>
        <Tooltip
          title={
            mappedAreas.length > 0 ? "Save mappings" : "Add mapping first..."
          }
        >
          <span>
            <Fab
              size="small"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={!mappedAreas.length > 0}
              onClick={handleSave}
            >
              <SaveIcon />
            </Fab>
          </span>
        </Tooltip>
      </div>
      <div>
        <Typography gutterBottom variant="h6" component="div" sx={{ mt: 4 }}>
          Defined Mappings for {props.srcCorticalArea}
        </Typography>
        <Divider />
        <Grid container spacing={2}>
          <Grid item xs={12} md={12} sx={{ mr: "10px" }}>
            <List>
              {mappedAreas.map((value, index) => {
                return (
                  <ListItem
                    divider
                    key={index}
                    secondaryAction={
                      <IconButton
                        edge="end"
                        aria-label="delete"
                        onClick={() => handleMappingDelete(index)}
                      >
                        <DeleteIcon />
                      </IconButton>
                    }
                  >
                    <ListItemText primary={index + 1} />
                    <ListItemText primary={value.dstArea} />
                    <ListItemText primary={value.rule} />
                    <ListItemText
                      primary="Morphology Scalar"
                      secondary={JSON.stringify(value.morphologyScalar)}
                    />
                    <ListItemText
                      primary="PSC Multiplier"
                      secondary={value.pscMultiplier}
                    />
                    <ListItemText
                      primary="Plasticity"
                      secondary={JSON.stringify(value.plasticity)}
                    />
                  </ListItem>
                );
              })}
            </List>
          </Grid>
        </Grid>
      </div>
    </>
  );
};

export default CorticalAreaMapForm;
