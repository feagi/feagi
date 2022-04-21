import { Route, Routes } from "react-router-dom";
import ChooseEnvironment from "./routes/ChooseEnvironment";
import CreateGenome from "./routes/CreateGenome";
import ChooseSensory from "./routes/ChooseSensory";
import CorticalMapping from "./routes/CorticalMapping";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  return (
    <>
      <ResponsiveAppBar />
      <Routes>
        <Route path="/environment" element={<ChooseEnvironment />} />
        <Route path="/genome" element={<CreateGenome />} />
        <Route path="/brain/mapping" element={<CorticalMapping />} />
        <Route path="/genome/sensory" element={<ChooseSensory />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
