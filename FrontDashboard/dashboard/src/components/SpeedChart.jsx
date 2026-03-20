import {
  LineChart, Line, XAxis, YAxis, Tooltip, CartesianGrid
} from "recharts";

const SpeedChart = ({ data }) => {
  return (
    <div>
      <h2>Speed Graph</h2>

      <LineChart width={600} height={300} data={data}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="step" />
        <YAxis />
        <Tooltip />
        <Line dataKey="speed" />
      </LineChart>
    </div>
  );
};

export default SpeedChart;