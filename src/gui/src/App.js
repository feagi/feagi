import { Route, Routes, Navigate } from "react-router-dom";
import Environment from "./routes/Environment";
import Genome from "./routes/Genome";
import Mapping from "./routes/Mapping";
import Motor from "./routes/Motor";
import Sensory from "./routes/Sensory";
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
        <Route path="/brain/mapping" element={<Mapping />} />
        <Route path="/brain/motor" element={<Motor />} />
        <Route path="/brain/sensory" element={<Sensory />} />
        <Route path="/monitoring" element={<MonitoringDashboard />} />
      </Routes>
      <ProgressStepper />
    </>
  );
}

export default App;
