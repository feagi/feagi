import React, { useState, useEffect } from "react";
import CircularProgress from "@mui/material/CircularProgress";
import Stack from "@mui/material/Stack";
import Iframe from "react-iframe";

const MonitoringDashboard = () => {
  const [frameHeight, setFrameHeight] = useState("");
  const [godotFrameLoaded, setGodotFrameLoaded] = useState(false);
  const [gazeboFrameLoaded, setGazeboFrameLoaded] = useState(false);

  useEffect(() => {
    const clientHeight = document.documentElement.scrollHeight;
    setTimeout(() => {
      setFrameHeight(clientHeight + "px");
    }, 100);
  }, []);

  const scrollHeightScaled =
    Math.round(document.documentElement.scrollHeight / 1.3) + "px";

  const handleGodotLoad = () => {
    setGodotFrameLoaded(true);
  };

  const handleGazeboLoad = () => {
    setGazeboFrameLoaded(true);
  };

  return (
    <Stack
      direction="row"
      alignItems="center"
      justifyContent="center"
      spacing={1}
      sx={{ mt: 2, mb: 2, ml: 1, mr: 1 }}
    >
      <Iframe
        className="iframe"
        id="godotFrame"
        url="http://localhost:6081/"
        width="50%"
        height={scrollHeightScaled}
        onLoad={handleGodotLoad}
      />
      {/* {godotFrameLoaded ? (
        <iframe
          id="godotFrame"
          src="http://localhost:6081"
          width="50%"
          height={scrollHeightScaled}
          onLoad={handleGodotLoad}
        />
      ) : (
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            height: scrollHeightScaled,
            width: "50%",
          }}
        >
          <CircularProgress size="150px" />
        </div>
      )} */}
      <Iframe
        className="iframe"
        id="gazeboFrame"
        url="http://localhost:6080"
        width="50%"
        height={scrollHeightScaled}
        onLoad={handleGazeboLoad}
      />
      {/* <div
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          height: scrollHeightScaled,
          width: "50%",
        }}
      >
        {gazeboFrameLoaded ? (
          <Iframe
            id="gazeboFrame"
            url="http://localhost:6080"
            width="100%"
            height={scrollHeightScaled}
            onLoad={handleGazeboLoad}
          />
        ) : (
          <CircularProgress size="150px" />
        )}
      </div> */}
    </Stack>
  );
};

export default MonitoringDashboard;
