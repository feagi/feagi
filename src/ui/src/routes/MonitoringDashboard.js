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

  const scrollHeightScaled =
    Math.round(document.documentElement.scrollHeight / 1.3) + "px";

  return (
    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={6}
      sx={{ mt: 8, mb: 16, ml: 4, mr: 4 }}
    >
      <Iframe
        id="godotFrame"
        url="http://localhost:6081"
        width="100%"
        height={scrollHeightScaled}
      />
      <Iframe
        id="gazeboFrame"
        url="http://localhost:6080"
        width="100%"
        height={scrollHeightScaled}
      />
    </Stack>
  );
};

export default MonitoringDashboard;
