import { Route, Routes } from "react-router-dom";
import ChooseEnvironment from "./routes/ChooseEnvironment";
import CreateGenome from "./routes/CreateGenome";
import DefineCorticalAreas from "./routes/DefineCorticalAreas";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  return (
    <>
      <ResponsiveAppBar />
      <Routes>
        <Route path="/environment" element={<ChooseEnvironment />} />
        <Route path="/genome" element={<CreateGenome />} />
        <Route path="/brain/build" element={<DefineCorticalAreas />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
