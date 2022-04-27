import React, { useState } from "react";
import { DataGrid } from "@mui/x-data-grid";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import Typography from "@mui/material/Typography";
import InputLabel from "@mui/material/InputLabel";

const columns = [
  { field: "parameter", headerName: "Parameter", width: 155 },
  { field: "value", headerName: "Value", width: 80, editable: true },
  { field: "description", headerName: "Description", width: 350 },
];

const rows = [
  {
    id: 1,
    parameter: "Neuron density",
    value: "",
    description: "Number (int) of neurons per voxel",
  },
  {
    id: 2,
    parameter: "Firing threshold",
    value: "",
    description: "Threshold to achieve action potential",
  },
  {
    id: 3,
    parameter: "Post-synaptic current",
    value: "",
    description: "Number (int) of neurons per voxel",
  },
];

const CorticalAreaForm = () => {
  // const [labelValue, setLabelValue] = useState("");
  // const [positionXValue, setPositionXValue] = useState("");
  // const [positionYValue, setPositionYValue] = useState("");
  // const [positionZValue, setPositionZValue] = useState("");
  // const [dimensionXValue, setDimensionXValue] = useState("");
  // const [dimensionYValue, setDimensionYValue] = useState("");
  // const [dimensionZValue, setDimensionZValue] = useState("");

  const [gridRows, setGridRows] = useState(rows);

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
      <Stack
        direction="row"
        alignItems="center"
        // justifyContent="center"
        spacing={2}
        sx={{ m: 1 }}
      >
        <InputLabel sx={{ width: "80px" }}>Label</InputLabel>
        <TextField
          id="filled-basic"
          label="cortical area name..."
          variant="filled"
          sx={{ width: "330px" }}
        />
      </Stack>
      <Stack
        direction="row"
        alignItems="center"
        // justifyContent="center"
        spacing={2}
        sx={{ m: 1 }}
      >
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
      <Stack
        direction="row"
        alignItems="center"
        // justifyContent="center"
        spacing={2}
        sx={{ m: 1 }}
      >
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

export default CorticalAreaForm;
