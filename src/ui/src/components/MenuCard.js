import React from "react";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import { CardActionArea } from "@mui/material";

const MenuCard = (props) => {
  return (
    <Card sx={{ maxWidth: 345, m: 2, mx: "auto" }}>
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
  );
};

export default MenuCard;
