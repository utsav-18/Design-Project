import {
  LineChart, Line, XAxis, YAxis, Tooltip
} from "recharts";

const GapChart = ({ data }) => {
  return (
    <div>
      <h2>Gap vs Safe Distance</h2>

      <LineChart width={600} height={300} data={data}>
        <XAxis dataKey="step" />
        <YAxis />
        <Tooltip />
        <Line dataKey="gap" />
        <Line dataKey="safe" />
      </LineChart>
    </div>
  );
};

export default GapChart;