import React, { useState } from "react";
import Button from "@mui/material/Button";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import { styled } from "@mui/material/styles";
import FeagiAPI from "../services/FeagiAPI";

const Input = styled("input")({
  display: "none",
});

const GenomeUploadCard = (props) => {
  const handleFileUpload = (event) => {
    try {
      let fileData = { file: event.target.files[0] };
      FeagiAPI.postGenomeFile(fileData);
    } catch (error) {
      console.error(error);
      throw error;
    }
  };

  return (
    <Card sx={{ width: 345, height: 330 }}>
      {props.image}
      <CardContent>
        <Typography gutterBottom variant="h5" component="div">
          {props.label}
        </Typography>
        <Typography variant="body2" color="text.secondary">
          {props.info}
        </Typography>
        <label htmlFor="contained-button-file">
          <Input
            accept="*.py"
            id="contained-button-file"
            type="file"
            onChange={handleFileUpload}
          />
          <Button variant="contained" component="span">
            Upload
          </Button>
        </label>
      </CardContent>
    </Card>
  );
};

export default GenomeUploadCard;
