import React from "react";
import { styled } from "@mui/material/styles";
import Card from "@mui/material/Card";
import Button from "@mui/material/Button";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";

const Input = styled("input")({
  display: "none",
});

const GenomeUploadCard = (props) => {
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
            multiple
            type="file"
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
