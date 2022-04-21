import React from "react";
import Stack from "@mui/material/Stack";
import { GiDna2 } from "react-icons/gi";
import { MdUploadFile } from "react-icons/md";
import { Link } from "react-router-dom";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import GenomeUploadCard from "../components/GenomeUploadCard";

const CreateGenome = () => {
  return (
    <>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 6 }}
      >
        <Item>
          <label htmlFor="regular-card">
            <MenuCard
              image={<GiDna2 size={200} />}
              label="Create Genome"
              info="Create a seed genome by defining desired sensorimotor capabilities, corresponding cortical areas and their connections."
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="upload-card">
            <GenomeUploadCard
              image={<MdUploadFile size={200} />}
              label="Use Existing Genome"
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default CreateGenome;
