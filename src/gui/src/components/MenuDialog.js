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
          {props.mode === "define" ? (
            <CorticalAreaEditForm
              setDialogOpen={setDialogOpen}
              definedSensory={props.definedSensory}
              setDefinedSensory={props.setDefinedSensory}
              definedMotor={props.definedMotor}
              setDefinedMotor={props.setDefinedMotor}
              corticalArea={props.label}
              type={props.type}
            />
          ) : (
            <CorticalAreaMapForm
              srcCorticalArea={props.label}
              definedMappings={props.definedMappings}
              setDefinedMappings={props.setDefinedMappings}
              availableMappingSensory={props.availableMappingSensory}
              availableMappingMotor={props.availableMappingMotor}
              setDialogOpen={setDialogOpen}
            />
          )}
        </DialogContent>
      </Dialog>
    </div>
  );
};

export default MenuDialog;
