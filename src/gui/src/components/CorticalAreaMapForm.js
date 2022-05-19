import React, { useState, useEffect } from "react";
import AddIcon from "@mui/icons-material/Add";
import DeleteIcon from "@mui/icons-material/Delete";
import Divider from "@mui/material/Divider";
import Fab from "@mui/material/Fab";
import FormControl from "@mui/material/FormControl";
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
import TextField from "@mui/material/TextField";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import FeagiAPI from "../services/FeagiAPI";

const CorticalAreaMapForm = (props) => {
  const [predefinedSynapseRules, setPredefinedSynapseRules] = useState({});
  const [selectedArea, setSelectedArea] = useState("");
  const [selectedRule, setSelectedRule] = useState("");
  const [mappedAreas, setMappedAreas] = useState([]);

  useEffect(() => {
    FeagiAPI.getBaselineMorphology().then((items) =>
      setPredefinedSynapseRules(items)
    );
  }, []);

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
        info: predefinedSynapseRules[selectedRule],
      },
    ]);
    setSelectedArea("");
    setSelectedRule("");
  };

  const handleMappingDelete = (index) => {
    let updatedMappings = mappedAreas;
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
        <FormControl sx={{ m: 2, minWidth: 120 }}>
          <InputLabel id="dest-select-label">Cortical Area</InputLabel>
          <Select
            labelId="dest-select-label"
            id="dest-select"
            value={selectedArea}
            label="Cortical Area"
            onChange={handleAreaChange}
            sx={{ width: "250px" }}
          >
            {props.availableMappingSensory
              .concat(props.availableMappingMotor)
              .map((area) => {
                return (
                  <MenuItem key={area} value={area}>
                    {area}
                  </MenuItem>
                );
              })}
          </Select>
          <FormHelperText>Required</FormHelperText>
        </FormControl>
        <FormControl sx={{ m: 2, minWidth: 120 }}>
          <InputLabel id="rule-select-label">Pre-defined Rule</InputLabel>
          <Select
            labelId="rule-select-label"
            id="rule-select"
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
          <FormHelperText>Required</FormHelperText>
        </FormControl>
        <FormControl sx={{ m: 2, minWidth: 200 }}>
          <TextField
            disabled
            id="rule-def-field"
            label={
              selectedRule
                ? JSON.stringify(predefinedSynapseRules[selectedRule])
                : "Rule info"
            }
            variant="outlined"
            sx={{ width: "250px" }}
          />
        </FormControl>
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
            >
              <AddIcon onClick={handleAdd} />
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
            >
              <SaveIcon onClick={handleSave} />
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
          <Grid item xs={12} md={8}>
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
