import React, { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { AiOutlineCar } from "react-icons/ai";
import { BiDna } from "react-icons/bi";
import { ImBrightnessContrast } from "react-icons/im";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import Item from "../components/Item";
import MenuCard from "../components/MenuCard";
import FeagiAPI from "../services/FeagiAPI";
import {Img} from 'react-image'

const DefaultMaps = (props) => {
  let navigate = useNavigate();

  const handleMapSelection = (type) => {
      FeagiAPI.postRobotModel({
        floor_img_file: type,
        floor_img_file_path: "./src/evo/defaults/maps/"
       });
    navigate("/robot/defaults");
  };

  return (
    <>
      <Typography
        variant="h4"
        align="center"
        sx={{ p: 4, mt: 8 }}
        component="div"
      >
        Select an Environment Map
      </Typography>
      <Stack
        direction="row"
        alignItems="center"
        justifyContent="center"
        spacing={2}
        sx={{ m: 2 }}
      >
        <Item>
          <label htmlFor="map-card">
            <MenuCard
              image={<Img src={require('../assets/map1.png')} width="150" height="150" />}
              label="Map 1"
              onClick={() => handleMapSelection("map1.png")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="map-card">
            <MenuCard
              image={<Img src={require('../assets/map2.png')} width="150" height="150" />}
              label="Map 2"
              onClick={() => handleMapSelection("map2.png")}
              changeColorOnClick={false}
            />
          </label>
        </Item>
        <Item>
          <label htmlFor="map-card">
            <MenuCard
              image={<Img src={require('../assets/map3.png')} width="150" height="150" />}
              label="Map 3"
              onClick={() =>
                handleMapSelection("map3.png")
              }
              changeColorOnClick={false}
            />
          </label>
        </Item>
      </Stack>
    </>
  );
};

export default DefaultMaps;
