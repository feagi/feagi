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
          width: "260px",
          height: "260px",
          backgroundColor: props.grayedOut
            ? "lightgray"
            : !props.changeColorOnClick
            ? null
            : !clicked
            ? null
            : "lightblue",
        }}
      >
        <CardActionArea>
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
        </CardActionArea>
      </Card>
    </div>
  );
};

export default MenuCard;
