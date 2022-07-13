import React, { useState } from "react";
import { Navigate } from "react-router-dom";
import Button from "@mui/material/Button";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import { styled } from "@mui/material/styles";
import FeagiAPI from "../services/FeagiAPI";
import Typography from "@mui/material/Typography";
import CardActionArea from "@mui/material/CardActionArea";

const Input = styled("input")({
  display: "none",
});

const GenomeUploadCard = (props) => {
  const [fileUploaded, setFileUploaded] = useState(false);

  const sendToMonitoringDashboard = () => {
    return <Navigate to="/monitoring" />;
  };

  const handleFileUpload = (event) => {
    try {
      let fileData = { file: event.target.files[0] };
      FeagiAPI.postGenomeFile(fileData);
      setFileUploaded(true);
    } catch (error) {
      console.error(error);
      throw error;
    }
  };

  return (
    <div onClick={props.onClick ? (e) => props.onClick(e, props.label) : null}>
      {fileUploaded ? (
        sendToMonitoringDashboard()
      ) : (
        <Card
            // onClick={handleFileUpload}
            sx={{
              width: "260px",
              height: "260px",
              backgroundColor: props.grayedOut
                ? "lightgray"
                : !props.changeColorOnClick
                // ? null
                // : !clicked
                ? null
                : "lightblue",
        }}
      >
          <CardActionArea>
            <label htmlFor="contained-button-file">
            <Input
              accept=".py"
              id="contained-button-file"
              type="file"
              onChange={handleFileUpload}
            />
            <CardContent>
              {props.image}
              <Typography
                gutterBottom
                variant="h5"
                component="div"
                sx={{ mt: 4 }}
              >
                {props.label}
              </Typography>
              <Typography variant="body2" color="text.secondary">
                {props.info}
              </Typography>
            </CardContent>
            </label>
          </CardActionArea>
        </Card>
      )}
    </div>
  );
};

export default GenomeUploadCard;
