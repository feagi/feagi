import React from "react";
import { useNavigate } from "react-router-dom";
import Chip from "@mui/material/Chip";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { MdArchitecture } from "react-icons/md";
import { GiDna2 } from "react-icons/gi";
import { MdUploadFile } from "react-icons/md";
import { GiRegeneration } from "react-icons/gi";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import GenomeUploadCard from "../components/GenomeUploadCard";
import Tooltip from "@mui/material/Tooltip";
import FeagiAPI from "../services/FeagiAPI";

const GenomeMode = () => {
  let navigate = useNavigate();

  const handleDefaultClick = () => {
    navigate("/genome/defaults");
  };

  const handleCreateGenomeClick = () => {
    navigate("/brain/sensorimotor");
  };

  const handleAutopilot = (type) => {
    FeagiAPI.postAutopilot({
    });
    navigate("/monitoring");
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select Genome Mode
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 2 }}
      >
        <Item>
          <label htmlFor="genome-card">
            <MenuCard
              image={<GiDna2 size={150} />}
              label="Sample Genomes"
              onClick={handleDefaultClick}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="genome-upload-card">
            <GenomeUploadCard
              image={<MdUploadFile size={150} />}
              label="Upload Genome"
            />
          </label>
        </Item>
        <Tooltip title="Coming soon...">
        <Item sx={{ position: "relative" }}>
          <label htmlFor="genome-create-card">
            <MenuCard
              image={<MdArchitecture size={150} />}
              label="Design Genome"
              // onClick={handleCreateGenomeClick}
              changeColorOnClick={false}
              grayedOut={true}
            />
          </label>
          {/*<Chip*/}
          {/*  label="Coming soon!"*/}
          {/*  color="primary"*/}
          {/*  sx={{*/}
          {/*    position: "absolute",*/}
          {/*    bottom: "-15%",*/}
          {/*    right: "31%",*/}
          {/*  }}*/}
          {/*/>*/}
        </Item>
        </Tooltip>
        <Item>
          <label htmlFor="genome-card">
            <MenuCard
              image={<GiRegeneration size={150} />}
              label="Autopilot"
              onClick={handleAutopilot}
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default GenomeMode;
