import React, { useState } from "react";
import Button from "@mui/material/Button";
import MenuItem from "@mui/material/MenuItem";
import EditIcon from "@mui/icons-material/Edit";
import KeyboardArrowDownIcon from "@mui/icons-material/KeyboardArrowDown";
import StyledMenu from "./StyledMenu";

const MenuButton = (props) => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);

  const handleMenuClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleMenuItemClick = () => {
    setAnchorEl(null);
    props.setDialogOpen(true);
  };
  const handleMenuClose = () => {
    setAnchorEl(null);
  };

  return (
    <div>
      <Button
        id="customized-button"
        aria-controls={open ? "demo-customized-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        variant="contained"
        disableElevation
        onClick={handleMenuClick}
        endIcon={<KeyboardArrowDownIcon />}
        sx={{ width: "175px" }}
      >
        {props.label}
      </Button>
      <StyledMenu
        id="customized-menu"
        MenuListProps={{
          "aria-labelledby": "demo-customized-button",
        }}
        anchorEl={anchorEl}
        open={open}
        onClose={handleMenuClose}
      >
        <MenuItem onClick={handleMenuItemClick} disableRipple>
          <EditIcon />
          Edit
        </MenuItem>
      </StyledMenu>
    </div>
  );
};

export default MenuButton;
