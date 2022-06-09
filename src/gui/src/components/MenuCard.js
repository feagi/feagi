import React, { useState } from "react";
import Card from "@mui/material/Card";
import CardActionArea from "@mui/material/CardActionArea";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";

const MenuCard = (props) => {
  const [clicked, setClicked] = useState(false);

  const handleClick = () => {
    setClicked(!clicked);
  };

  return (
    <div onClick={props.onClick ? (e) => props.onClick(e, props.label) : null}>
      <Card
        onClick={handleClick}
        sx={{
          width: "245px",
          height: "230px",
          backgroundColor: !clicked ? null : "lightblue",
        }}
      >
        <CardActionArea>
          {props.image}
          <CardContent>
            <Typography gutterBottom variant="h5" component="div">
              {props.label}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              {props.info}
            </Typography>
          </CardContent>
        </CardActionArea>
      </Card>
    </div>
  );
};

export default MenuCard;
