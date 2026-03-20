import { useState } from "react";
import Upload from "./components/Upload";
import Dashboard from "./components/Dashboard";

function App() {
  const [data, setData] = useState([]);

  return (
    <div className="container">
      <h1  className="title">🚗 Collision Dashboard</h1>

      <Upload setData={setData} />

      <Dashboard data={data} />
    </div>
  );
}

export default App;