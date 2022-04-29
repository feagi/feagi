import React, { useState } from "react";
import Dialog from "@mui/material/Dialog";
import DialogContent from "@mui/material/DialogContent";
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
        maxWidth="md"
      >
        <DialogContent>
          {props.type === "cortical" ? (
            <CorticalAreaEditForm corticalArea={props.label} />
          ) : (
            <CorticalAreaMapForm corticalArea={props.label} />
          )}
        </DialogContent>
      </Dialog>
    </div>
  );
};

export default MenuDialog;
