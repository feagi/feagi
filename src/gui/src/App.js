import { Route, Routes, Navigate } from "react-router-dom";
import Environment from "./routes/Environment";
import Genome from "./routes/Genome";
import DefineMapping from "./routes/DefineMapping";
import DefineMotor from "./routes/DefineMotor";
import DefineSensory from "./routes/DefineSensory";
import MonitoringDashboard from "./routes/MonitoringDashboard";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import ProgressStepper from "./components/ProgressStepper";

function App() {
  return (
    <>
      <ResponsiveAppBar />
      <Routes>
        <Route path="/" element={<Navigate replace to="/environment" />} />
        <Route path="/environment" element={<Environment />} />
        <Route path="/genome" element={<Genome />} />
        <Route path="/brain/mapping" element={<DefineMapping />} />
        <Route path="/brain/motor" element={<DefineMotor />} />
        <Route path="/brain/sensory" element={<DefineSensory />} />
        <Route path="/monitoring" element={<MonitoringDashboard />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
