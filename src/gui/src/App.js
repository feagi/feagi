import { Route, Routes, Navigate } from "react-router-dom";
import ChooseEnvironment from "./routes/ChooseEnvironment";
import CreateGenome from "./routes/CreateGenome";
import DefineCorticalAreas from "./routes/DefineCorticalAreas";
import MonitoringDashboard from "./routes/MonitoringDashboard";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  return (
    <>
      <ResponsiveAppBar />
      <Routes>
        <Route path="/" element={<Navigate replace to="/environment" />} />
        <Route path="/environment" element={<ChooseEnvironment />} />
        <Route path="/genome" element={<CreateGenome />} />
        <Route path="/brain" element={<DefineCorticalAreas />} />
        <Route path="/monitoring" element={<MonitoringDashboard />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
