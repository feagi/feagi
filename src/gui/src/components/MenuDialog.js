import React, { useState } from "react";
import Dialog from "@mui/material/Dialog";
import DialogContent from "@mui/material/DialogContent";
import DialogTitle from "@mui/material/DialogTitle";
import CorticalAreaEditForm from "./CorticalAreaEditForm";
import CorticalAreaMapForm from "./CorticalAreaMapForm";
import MenuButton from "./MenuButton";

const MenuDialog = (props) => {
  const [dialogOpen, setDialogOpen] = useState(false);

  const handleDialogClose = () => {
    setDialogOpen(false);
  };

  return (
    <div>
      <MenuButton label={props.label} setDialogOpen={setDialogOpen} />
      <Dialog
        open={dialogOpen}
        onClose={handleDialogClose}
        fullWidth
        maxWidth="sm"
      >
        <DialogTitle>Cortical Area Definition</DialogTitle>
        <DialogContent>
          {props.type === "cortical" ? (
            <CorticalAreaEditForm />
          ) : (
            <CorticalAreaMapForm />
          )}
        </DialogContent>
      </Dialog>
    </div>
  );
};

export default MenuDialog;
