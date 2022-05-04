import React, { useState, useEffect } from "react";
import { DataGrid } from "@mui/x-data-grid";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import Typography from "@mui/material/Typography";
import InputLabel from "@mui/material/InputLabel";
import FeagiAPI from "../services/FeagiAPI";

const columns = [
  { field: "parameter", headerName: "Parameter", width: 150 },
  { field: "value", headerName: "Value", width: 150, editable: true },
  { field: "description", headerName: "Description", width: 560 },
];

const rows = [];

const CorticalAreaEditForm = (props) => {
  // const [labelValue, setLabelValue] = useState("");
  // const [positionXValue, setPositionXValue] = useState("");
  // const [positionYValue, setPositionYValue] = useState("");
  // const [positionZValue, setPositionZValue] = useState("");
  // const [dimensionXValue, setDimensionXValue] = useState("");
  // const [dimensionYValue, setDimensionYValue] = useState("");
  // const [dimensionZValue, setDimensionZValue] = useState("");

  const [gridRows, setGridRows] = useState(rows);
  const [corticalGenes, setCorticalGenes] = useState({});

  useEffect(() => {
    FeagiAPI.getBaselineCorticalGenes().then((items) =>
      setCorticalGenes(items)
    );
  }, []);

  Object.keys(corticalGenes).forEach((key, index) => {
    rows.push({
      id: `${index + 1}`,
      parameter: `${corticalGenes[key]}`,
      value: "",
      description: `${key}`,
    });
  });

  const handleCellEditCommit = ({ id, field, value }) => {
    if (field === "value") {
      const updatedRows = rows.map((row) => {
        if (row.id === id) {
          row.value = value;
          return { ...row };
        }
        return row;
      });
      setGridRows(updatedRows);
    }
  };

  return (
    <div>
      <Typography gutterBottom variant="h5" component="div" sx={{ mb: 3 }}>
        {props.corticalArea} cortical area properties
      </Typography>
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Label</InputLabel>
        <TextField
          id="filled-basic"
          label="cortical area name..."
          variant="filled"
          sx={{ width: "330px" }}
        />
      </Stack>
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Position</InputLabel>
        <TextField
          id="filled-basic"
          label="X"
          variant="filled"
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic"
          label="Y"
          variant="filled"
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic"
          label="Z"
          variant="filled"
          sx={{ width: "100px" }}
        />
      </Stack>
      <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
        <InputLabel sx={{ width: "80px" }}>Dimension</InputLabel>
        <TextField
          id="filled-basic"
          label="X"
          variant="filled"
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic"
          label="Y"
          variant="filled"
          sx={{ width: "100px" }}
        />
        <TextField
          id="filled-basic"
          label="Z"
          variant="filled"
          sx={{ width: "100px" }}
        />
      </Stack>
      <Typography
        gutterBottom
        variant="h6"
        component="div"
        sx={{ justifyContent: "center", mt: 4 }}
      >
        Neuron Parameters (advanced options)
      </Typography>
      <DataGrid
        onCellEditCommit={handleCellEditCommit}
        columns={columns}
        rows={rows}
        sx={{ height: "275px" }}
      />
    </div>
  );
};

export default CorticalAreaEditForm;
