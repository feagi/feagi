import { Route, Routes } from "react-router-dom";
import ChooseEnv from "./routes/ChooseEnv";
import CreateGenome from "./routes/CreateGenome";
import CorticalMapping from "./routes/CorticalMapping";

import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  return (
    <>
      <ResponsiveAppBar />
      <ProgressStepper />
      <Routes>
        <Route path="/environment" element={<ChooseEnv />} />
        <Route path="/genome" element={<CreateGenome />} />
        <Route path="/brain/mapping" element={<CorticalMapping />} />
      </Routes>
    </>
  );
}

export default App;
