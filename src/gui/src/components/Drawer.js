import React, { useState } from "react";
import Box from "@mui/material/Box";
import Drawer from "@mui/material/Drawer";
import List from "@mui/material/List";
import Divider from "@mui/material/Divider";
import ListItem from "@mui/material/ListItem";
import ListItemIcon from "@mui/material/ListItemIcon";
import ListItemText from "@mui/material/ListItemText";
import AccessAlarmIcon from "@mui/icons-material/AccessAlarm";
import CameraAltIcon from "@mui/icons-material/CameraAlt";

export default function TemporaryDrawer() {
  const [drawerState, setDrawerState] = useState(true);

  const toggleDrawer = (open) => (event) => {
    if (
      event.type === "keydown" &&
      (event.key === "Tab" || event.key === "Shift")
    ) {
      return;
    }
    setDrawerState(open);
  };

  const list = () => (
    <Box
      sx={{ width: 250 }}
      role="presentation"
      onClick={toggleDrawer(false)}
      onKeyDown={toggleDrawer(false)}
    >
      <List>
        {["Change Burst Frequency", "Take Connectome Snapshot"].map(
          (text, index) => (
            <ListItem button key={text}>
              <ListItemIcon>
                {index % 2 === 0 ? <AccessAlarmIcon /> : <CameraAltIcon />}
              </ListItemIcon>
              <ListItemText primary={text} />
            </ListItem>
          )
        )}
      </List>
      <Divider />
    </Box>
  );

  return (
    <div>
      <Drawer anchor="left" open={drawerState} onClose={toggleDrawer(false)}>
        {list}
      </Drawer>
    </div>
  );
}
