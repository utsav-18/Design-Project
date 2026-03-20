import { useState } from "react";
import SummaryCards from "./SummaryCard";
import SpeedChart from "./SpeedChart";
import GapChart from "./GapChart";
import Alerts from "./Alerts";

const Dashboard = ({ data }) => {
  const [vehicle, setVehicle] = useState("leader");

  if (!data || data.length === 0) return null;

  // 🔥 FILTER
  const vehicleData = data.filter((d) => d.veh === vehicle);

  // 🔥 CLEAN
  const cleanData = vehicleData.map((d) => ({
    step: Number(d.step),
    speed: Number(d.speed_mps),
    gap: Number(d.gap_m || 0),
    safe: Number(d.safe_m || 0),
    action: d.action,
  }));

  return (
    <div>

      {/* 🔥 TOP CONTROL BAR */}
      <div className="card" style={{ marginBottom: "20px" }}>
        <h2 style={{ marginBottom: "10px" }}>⚙️ Controls</h2>

        <select
          value={vehicle}
          onChange={(e) => setVehicle(e.target.value)}
        >
          <option value="leader">Leader</option>
          <option value="main_flow_e_w.1">Vehicle 1</option>
          <option value="main_flow_e_w.2">Vehicle 2</option>
        </select>
      </div>

      {/* 🔥 SUMMARY */}
      <SummaryCards data={cleanData} />

      {/* 🔥 SPEED GRAPH */}
      <div className="card chart">
        <h2>📈 Speed Graph</h2>
        <SpeedChart data={cleanData} />
      </div>

      {/* 🔥 GAP GRAPH */}
      <div className="card chart">
        <h2>📉 Gap vs Safe Distance</h2>
        <GapChart data={cleanData} />
      </div>

      {/* 🔥 ALERTS */}
      <Alerts data={cleanData} />

    </div>
  );
};

export default Dashboard;