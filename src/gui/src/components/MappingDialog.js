import React, { useState } from "react";
import Dialog from "@mui/material/Dialog";
import DialogContent from "@mui/material/DialogContent";
import CorticalAreaMapForm from "./CorticalAreaMapForm";
import MappingButton from "./MappingButton";

const MappingDialog = (props) => {
  const [dialogOpen, setDialogOpen] = useState(false);

  const handleDialogClose = () => {
    setDialogOpen(false);
  };

  return (
    <div>
      <MappingButton label={props.label} setDialogOpen={setDialogOpen} />
      <Dialog
        open={dialogOpen}
        onClose={handleDialogClose}
        fullWidth
        maxWidth="md"
      >
        <DialogContent>
          <CorticalAreaMapForm
            srcCorticalArea={props.label}
            definedMappings={props.definedMappings}
            setDefinedMappings={props.setDefinedMappings}
            availableMappingAreas={props.availableMappingAreas}
            defaultMorphologyScalarX={props.defaultMorphologyScalarX}
            defaultMorphologyScalarY={props.defaultMorphologyScalarY}
            defaultMorphologyScalarZ={props.defaultMorphologyScalarZ}
            defaultPscMultiplier={props.defaultPscMultiplier}
            defaultPlasticityFlag={props.defaultPlasticityFlag}
            defaultSynapseRules={props.defaultSynapseRules}
            setDialogOpen={setDialogOpen}
          />
        </DialogContent>
      </Dialog>
    </div>
  );
};

export default MappingDialog;
