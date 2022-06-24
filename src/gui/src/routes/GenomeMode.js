import React from "react";
import { useNavigate } from "react-router-dom";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { BiDna } from "react-icons/bi";
import { GiDna2 } from "react-icons/gi";
import { MdUploadFile } from "react-icons/md";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import GenomeUploadCard from "../components/GenomeUploadCard";

const GenomeMode = () => {
  let navigate = useNavigate();

  const handleDefaultClick = () => {
    navigate("/genome/defaults");
  };

  const handleCreateGenomeClick = () => {
    navigate("/brain/sensorimotor");
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
              image={<BiDna size={150} />}
              label="Default Genomes"
              onClick={handleDefaultClick}
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
        <Item>
          <label htmlFor="genome-card">
            <MenuCard
              image={<GiDna2 size={150} />}
              label="Create Genome"
              onClick={handleCreateGenomeClick}
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default GenomeMode;
