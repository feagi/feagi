import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import AddIcon from "@mui/icons-material/Add";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import Button from "@mui/material/Button";
import CloseIcon from "@mui/icons-material/Close";
import DeleteIcon from "@mui/icons-material/Delete";
import Dialog from "@mui/material/Dialog";
import DialogActions from "@mui/material/DialogActions";
import DialogContent from "@mui/material/DialogContent";
import DownloadIcon from "@mui/icons-material/Download";
import EditIcon from "@mui/icons-material/Edit";
import Fab from "@mui/material/Fab";
import fileDownload from "js-file-download";
import IconButton from "@mui/material/IconButton";
import InputLabel from "@mui/material/InputLabel";
import Paper from "@mui/material/Paper";
import Stack from "@mui/material/Stack";
import { styled } from "@mui/material/styles";
import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableHead from "@mui/material/TableHead";
import TablePagination from "@mui/material/TablePagination";
import TableRow from "@mui/material/TableRow";
import UploadIcon from "@mui/icons-material/Upload";
import TextField from "@mui/material/TextField";
import Typography from "@mui/material/Typography";
import CorticalAreaEditForm from "../components/CorticalAreaEditForm";
import { genomeBase } from "../constants/genome";
import FeagiAPI from "../services/FeagiAPI";

const Input = styled("input")({
  display: "none",
});

const CorticalAreaEditor = (props) => {
  const [listedAreas, setListedAreas] = useState([
    ...props.selectedSensory,
    ...props.selectedMotor,
    ...props.customAreas,
  ]);
  const [formVisible, setFormVisible] = useState(false);
  const [activeArea, setActiveArea] = useState("");
  const [rowsPerPage, setRowsPerPage] = useState(10);
  const [page, setPage] = useState(0);
  const [addedAreaName, setAddedAreaName] = useState("");
  const [areaNameDialogVisible, setAreaNameDialogVisible] = useState(false);

  const handleAreaDelete = (area, index) => {
    let paginatedIndex = index + page * rowsPerPage;
    let updatedAreas = [...listedAreas];
    updatedAreas.splice(paginatedIndex, 1);
    setListedAreas(updatedAreas);
    if (area in props.definedAreas) {
      let updatedDefinedAreas = { ...props.definedAreas };
      delete updatedDefinedAreas[area];
      props.setDefinedAreas(updatedDefinedAreas);
    }
  };

  const handleAreaAdd = () => {
    setAreaNameDialogVisible(true);
  };

  const handleAreaEdit = (area) => {
    setActiveArea(area);
    setFormVisible(true);
  };

  const handleFormDialogClose = () => {
    setFormVisible(false);
  };

  const handleSave = () => {
    let genome = genomeBase;
    let blueprintShell = {};
    for (const area in props.definedAreas) {
      for (const gene in props.definedAreas[area]) {
        blueprintShell[gene] = props.definedAreas[area][gene];
      }
    }
    genome["blueprint"] = blueprintShell;
    const fileName = `genome_${Date.now()}.json`;
    fileDownload(JSON.stringify(genome), fileName);
  };

  let navigate = useNavigate();
  const handleNext = () => {
    navigate("/brain/mapping");
  };

  const handleBack = () => {
    navigate("/brain/sensorimotor");
  };

  const handleChangePage = (event, newPage) => {
    setPage(newPage);
  };

  const handleChangeRowsPerPage = (event) => {
    setRowsPerPage(parseInt(event.target.value, 10));
    setPage(0);
  };

  const handleAreaNameDialogClose = () => {
    setListedAreas([...listedAreas, addedAreaName]);
    props.setCustomAreas([...props.customAreas, addedAreaName]);
    setAreaNameDialogVisible(false);
  };

  const handleAreaNameDialogXout = () => {
    setAreaNameDialogVisible(false);
  };

  const handleInputClick = (event) => {
    event.target.value = "";
  };

  const handleGenomeUpload = async (event) => {
    const genomeFileData = await FeagiAPI.postGenomeFileEdit({
      file: event.target.files[0],
    });

    const genomeBlueprint = genomeFileData["blueprint"];

    let loadedAreas = {};
    let groupIdAreaMap = {};
    const loadedAreaNameGenes = Object.entries(genomeBlueprint).filter((gene) =>
      gene[0].includes("name-")
    );

    for (const areaNameGene of loadedAreaNameGenes) {
      loadedAreas[genomeBlueprint[areaNameGene[0]]] = {};
      groupIdAreaMap[areaNameGene[0].slice(9, 15)] = areaNameGene[1];
    }

    for (const gene in genomeBlueprint) {
      loadedAreas[groupIdAreaMap[gene.slice(9, 15)]][gene] =
        genomeBlueprint[gene];
    }

    setListedAreas(Object.keys(loadedAreas));
    props.setDefinedAreas(loadedAreas);
  };

  const showForm = () => {
    return (
      <Dialog
        open={formVisible}
        onClose={handleFormDialogClose}
        fullWidth
        maxWidth="md"
      >
        <DialogContent>
          <CorticalAreaEditForm
            setDialogOpen={setFormVisible}
            definedSensory={props.definedSensory}
            setDefinedSensory={props.setDefinedSensory}
            definedMotor={props.definedMotor}
            setDefinedMotor={props.setDefinedMotor}
            definedAreas={props.definedAreas}
            setDefinedAreas={props.setDefinedAreas}
            defaultCorticalGenes={props.defaultCorticalGenes}
            corticalArea={activeArea}
          />
        </DialogContent>
      </Dialog>
    );
  };

  const promptUserForAreaName = () => {
    return (
      <Dialog open={areaNameDialogVisible} fullWidth maxWidth="sm">
        <DialogContent>
          <div style={{ display: "flex", justifyContent: "flex-end" }}>
            <IconButton sx={{ mb: 2 }} onClick={handleAreaNameDialogXout}>
              <CloseIcon />
            </IconButton>
          </div>
          <Stack direction="row" alignItems="center" spacing={2} sx={{ m: 1 }}>
            <InputLabel sx={{ width: "350px" }}>
              <Typography fontWeight="bold">
                Enter a name for the cortical area:{" "}
              </Typography>
            </InputLabel>
            <TextField
              id="filled-basic"
              variant="filled"
              onChange={(e) => setAddedAreaName(e.target.value)}
              sx={{ width: "330px" }}
            />
          </Stack>
          <DialogActions>
            <Button
              variant="contained"
              onClick={handleAreaNameDialogClose}
              disabled={!addedAreaName}
            >
              OK
            </Button>
          </DialogActions>
        </DialogContent>
      </Dialog>
    );
  };

  return (
    <>
      <Typography variant="h4" component="div" sx={{ mt: 4, ml: 4, mb: 1 }}>
        Define/Edit Cortical Areas
      </Typography>
      <TableContainer component={Paper} sx={{ mt: "20px" }}>
        <Table>
          <TableHead sx={{ backgroundColor: "lightgray" }}>
            <TableRow>
              <TableCell align="center">
                {<Typography variant="h5">Area</Typography>}
              </TableCell>
              <TableCell align="left">
                {<Typography variant="h5">Dimensions</Typography>}
              </TableCell>
              <TableCell align="left">
                {<Typography variant="h5">Position</Typography>}
              </TableCell>
              <TableCell align="center">
                {<Typography variant="h5">Management</Typography>}
              </TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {listedAreas
              .slice(page * rowsPerPage, page * rowsPerPage + rowsPerPage)
              .map((area, index) => (
                <TableRow hover key={index}>
                  <TableCell
                    align="center"
                    component="th"
                    scope="row"
                    style={{ width: "400px" }}
                  >
                    {<Typography>{area}</Typography>}
                  </TableCell>
                  <TableCell style={{ width: "500px" }}>
                    {
                      <Stack direction="row" alignItems="center" spacing={2}>
                        <TextField
                          id="filled-basic-dx"
                          label="X"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("bbx")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                        <TextField
                          id="filled-basic-dy"
                          label="Y"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("bby")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                        <TextField
                          id="filled-basic-dz"
                          label="Z"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("bbz")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                      </Stack>
                    }
                  </TableCell>
                  <TableCell align="center" style={{ width: "400px" }}>
                    {
                      <Stack direction="row" alignItems="center" spacing={2}>
                        <TextField
                          id="filled-basic-px"
                          label="X"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("rcordx")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                        <TextField
                          id="filled-basic-py"
                          label="Y"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("rcordy")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                        <TextField
                          id="filled-basic-pz"
                          label="Z"
                          variant="filled"
                          disabled
                          value={
                            area in props.definedAreas
                              ? props.definedAreas[area][
                                  Object.keys(props.definedAreas[area]).filter(
                                    (key) => key.includes("rcordz")
                                  )[0]
                                ]
                              : "none"
                          }
                          sx={{ width: "75px" }}
                          InputProps={{
                            inputProps: {
                              style: { textAlign: "center" },
                            },
                          }}
                        />
                      </Stack>
                    }
                  </TableCell>
                  <TableCell align="center">
                    {
                      <>
                        <Fab
                          size="small"
                          color="primary"
                          aria-label="add-icon"
                          sx={{ m: 2 }}
                          onClick={() => handleAreaEdit(area)}
                        >
                          <EditIcon />
                        </Fab>
                        <Fab
                          size="small"
                          color="primary"
                          aria-label="delete-icon"
                          sx={{ m: 2 }}
                          onClick={() => handleAreaDelete(area, index)}
                        >
                          <DeleteIcon />
                        </Fab>
                      </>
                    }
                  </TableCell>
                </TableRow>
              ))}
          </TableBody>
        </Table>
        <Fab
          size="medium"
          color="primary"
          aria-label="add"
          sx={{ mt: 7, ml: 2, mr: 2, mb: 2 }}
          onClick={handleAreaAdd}
        >
          <AddIcon />
        </Fab>
        <Fab
          size="medium"
          color="primary"
          aria-label="save"
          sx={{ mt: 7, ml: 2, mr: 2, mb: 2 }}
          disabled={!props.definedAreas}
          onClick={handleSave}
        >
          <DownloadIcon />
        </Fab>
        <label htmlFor="genome-upload">
          <Input
            id="genome-upload"
            accept=".json"
            type="file"
            onChange={handleGenomeUpload}
            onClick={handleInputClick}
          />
          <Fab
            id="genome-upload"
            component="span"
            size="medium"
            color="primary"
            sx={{ mt: 7, ml: 2, mr: 2, mb: 2 }}
          >
            <UploadIcon />
          </Fab>
        </label>
        <TablePagination
          rowsPerPageOptions={[5, 10, 25, 50, 100]}
          component="div"
          count={listedAreas.length}
          rowsPerPage={parseInt(rowsPerPage, 10)}
          page={page}
          onPageChange={handleChangePage}
          onRowsPerPageChange={handleChangeRowsPerPage}
        />
      </TableContainer>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 8 }}
      >
        <Fab
          size="large"
          color="primary"
          aria-label="add"
          sx={{ m: 1 }}
          onClick={handleBack}
        >
          <ArrowBackIcon />
        </Fab>
        <Fab
          size="large"
          color="primary"
          aria-label="add"
          sx={{ m: 1 }}
          disabled={
            Object.keys(props.definedAreas).length !== listedAreas.length
          }
          onClick={handleNext}
        >
          <ArrowForwardIcon />
        </Fab>
      </Stack>
      {formVisible && showForm()}
      {areaNameDialogVisible && promptUserForAreaName()}
    </>
  );
};

export default CorticalAreaEditor;
