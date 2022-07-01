import React, { useState } from "react";
import { Navigate } from "react-router-dom";
import Button from "@mui/material/Button";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import { styled } from "@mui/material/styles";
import FeagiAPI from "../services/FeagiAPI";

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
    <>
      {fileUploaded ? (
        sendToMonitoringDashboard()
      ) : (
        <Card sx={{ width: "260px", height: "260px" }}>
          {props.image}
          <CardContent>
            <label htmlFor="contained-button-file">
              <Input
                accept=".py"
                id="contained-button-file"
                type="file"
                onChange={handleFileUpload}
              />
              <Button variant="contained" component="span" sx={{ mt: 3 }}>
                Upload Genome
              </Button>
            </label>
          </CardContent>
        </Card>
      )}
    </>
  );
};

export default GenomeUploadCard;
