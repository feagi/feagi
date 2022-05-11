import React, { useState, useEffect } from "react";
import Divider from "@mui/material/Divider";
import Fab from "@mui/material/Fab";
import SaveIcon from "@mui/icons-material/Save";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import Tooltip from "@mui/material/Tooltip";
import Typography from "@mui/material/Typography";
import InputLabel from "@mui/material/InputLabel";
import FeagiAPI from "../services/FeagiAPI";

const CorticalAreaEditForm = (props) => {
  const [labelValue, setLabelValue] = useState(
    props.corticalArea.toLowerCase()
  );
  const [positionXValue, setPositionXValue] = useState("");
  const [positionYValue, setPositionYValue] = useState("");
  const [positionZValue, setPositionZValue] = useState("");
  const [dimensionXValue, setDimensionXValue] = useState("");
  const [dimensionYValue, setDimensionYValue] = useState("");
  const [dimensionZValue, setDimensionZValue] = useState("");
  const [corticalGenes, setCorticalGenes] = useState({});

  useEffect(() => {
    FeagiAPI.getBaselineCorticalGenes().then((items) =>
      setCorticalGenes(items)
    );
  }, []);

  const defaultGenes = {};
  Object.keys(corticalGenes).forEach((key) => {
    defaultGenes[corticalGenes[key][0]] = corticalGenes[key][1];
  });

  const assembleSensoryGenomeData = () => {
    const labelSlice = labelValue.slice(0, 3);
    const groupSlice = defaultGenes["cx-_group-t"][0];
    const genePrefix = `_____10c-${groupSlice}__${labelSlice}-`;

    const labelGene = genePrefix.concat("cx-__name-t");

    const positionXGene = genePrefix.concat("nx-rcordx-i");
    const positionYGene = genePrefix.concat("nx-rcordy-i");
    const positionZGene = genePrefix.concat("nx-rcordz-i");

    const dimensionXGene = genePrefix.concat("nx-___bbx-i");
    const dimensionYGene = genePrefix.concat("nx-___bby-i");
    const dimensionZGene = genePrefix.concat("nx-___bbz-i");

    let prefixedDefaultGenes = {};
    Object.keys(defaultGenes).forEach((key) => {
      prefixedDefaultGenes[genePrefix.concat(key)] = defaultGenes[key];
    });

    const prefixedGenes = {
      [labelGene]: labelValue,
      [positionXGene]: parseInt(positionXValue),
      [positionYGene]: parseInt(positionYValue),
      [positionZGene]: parseInt(positionZValue),
      [dimensionXGene]: parseInt(dimensionXValue),
      [dimensionYGene]: parseInt(dimensionYValue),
      [dimensionZGene]: parseInt(dimensionZValue),
      ...prefixedDefaultGenes,
    };

    return prefixedGenes;
  };

  const handleGeneValueChange = (event, gene) => {
    defaultGenes[gene] = event;
  };

  const handleSave = () => {
    const definedArea = assembleSensoryGenomeData();
    console.log(props.definedSensory);
    props.setDefinedSensory({
      ...props.definedSensory,
      [props.corticalArea.toLowerCase()]: definedArea,
    });
    props.setDialogOpen(false);
  };

  return (
    <>
      <Typography gutterBottom variant="h5" component="div" sx={{ mb: 1 }}>
        {props.corticalArea} cortical area properties
      </Typography>
      <Divider sx={{ mb: 2 }} />
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Label</InputLabel>
        <TextField
          id="filled-basic"
          label="cortical area name..."
          defaultValue={props.corticalArea.toLowerCase()}
          variant="filled"
          onChange={(e) => setLabelValue(e.target.value)}
          sx={{ width: "330px" }}
        />
      </Stack>
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Position</InputLabel>
        <TextField
          id="filled-basic-px"
          label="X"
          variant="filled"
          onChange={(e) => setPositionXValue(e.target.value)}
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic-py"
          label="Y"
          variant="filled"
          onChange={(e) => setPositionYValue(e.target.value)}
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic-pz"
          label="Z"
          variant="filled"
          onChange={(e) => setPositionZValue(e.target.value)}
          sx={{ width: "100px" }}
        />
      </Stack>
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Dimension</InputLabel>
        <TextField
          id="filled-basic-dx"
          label="X"
          variant="filled"
          onChange={(e) => setDimensionXValue(e.target.value)}
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic-dy"
          label="Y"
          variant="filled"
          onChange={(e) => setDimensionYValue(e.target.value)}
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic-dz"
          label="Z"
          variant="filled"
          onChange={(e) => setDimensionZValue(e.target.value)}
          sx={{ width: "100px" }}
        />
      </Stack>
      <Typography
        gutterBottom
        variant="h6"
        component="div"
        sx={{ justifyContent: "center", mt: 4, mb: 1 }}
      >
        Neuron Parameters (advanced options)
      </Typography>
      <Divider sx={{ mb: 2 }} />
      {Object.keys(corticalGenes).map((key, index) => {
        return (
          <Stack
            key={`stack-${index}`}
            direction="row"
            alignItems="center"
            spacing={2}
            sx={{ m: 1 }}
          >
            <InputLabel key={`input-${index}`} sx={{ width: "250px" }}>
              {key}
            </InputLabel>
            <TextField
              key={`field-${index}`}
              id={`filled-basic-${index}`}
              label={corticalGenes[key][0]}
              defaultValue={corticalGenes[key][1]}
              variant="filled"
              onChange={(e) =>
                handleGeneValueChange(e.target.value, corticalGenes[key][0])
              }
              sx={{ width: "150px" }}
            />
          </Stack>
        );
      })}
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 2 }}
      >
        <Tooltip title="Save">
          <span>
            <Fab
              size="large"
              color="primary"
              aria-label="add"
              sx={{ m: 1 }}
              disabled={
                !(
                  labelValue &&
                  positionXValue &&
                  positionYValue &&
                  positionZValue &&
                  dimensionXValue &&
                  dimensionYValue &&
                  dimensionZValue
                )
              }
            >
              <SaveIcon onClick={handleSave} />
            </Fab>
          </span>
        </Tooltip>
      </Stack>
    </>
  );
};

export default CorticalAreaEditForm;
