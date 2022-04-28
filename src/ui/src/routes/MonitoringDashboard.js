import React, { useState, useEffect } from "react";
import Stack from "@mui/material/Stack";
import Iframe from "react-iframe";

const MonitoringDashboard = () => {
  const [frameHeight, setFrameHeight] = useState("");

  useEffect(() => {
    const clientHeight = document.documentElement.scrollHeight;
    setTimeout(() => {
      setFrameHeight(clientHeight + "px");
    }, 100);
  }, []);

  return (
    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={6}
      sx={{ m: 4 }}
    >
      <Iframe
        id="godotFrame"
        url="http://localhost:6080"
        width="100%"
        height={frameHeight}
      />
      <Iframe
        id="gazeboFrame"
        url="http://localhost:6080"
        width="100%"
        height={frameHeight}
      />
    </Stack>
  );
};

export default MonitoringDashboard;
