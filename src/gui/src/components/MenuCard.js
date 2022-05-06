import React from "react";
import Card from "@mui/material/Card";
import CardActionArea from "@mui/material/CardActionArea";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";

const MenuCard = (props) => {
  return (
    <div onClick={props.onClick ? (e) => props.onClick(e, props.label) : null}>
      <Card sx={{ width: 345, height: 330 }}>
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
