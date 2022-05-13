import React from "react";
import Stack from "@mui/material/Stack";
import { BiDna } from "react-icons/bi";
import { GiDna2 } from "react-icons/gi";
import { MdUploadFile } from "react-icons/md";
import { Link } from "react-router-dom";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import GenomeUploadCard from "../components/GenomeUploadCard";
import FeagiAPI from "../services/FeagiAPI";

const CreateGenome = () => {
  const handleDefaultClick = () => {
    FeagiAPI.postDefaultGenome();
  };

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
          <Link to="/monitoring">
            <label htmlFor="genome-card">
              <MenuCard
                image={<BiDna size={150} />}
                label="Default Genome"
                onClick={handleDefaultClick}
              />
            </label>
          </Link>
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
          <Link to="/brain/sensory">
            <label htmlFor="genome-card">
              <MenuCard
                image={<GiDna2 size={150} />}
                label="Create Genome"
                // info="Create a seed genome by defining desired sensorimotor capabilities, corresponding cortical areas and their connections."
              />
            </label>
          </Link>
        </Item>
      </Stack>
    </>
  );
};

export default CreateGenome;
