const Alerts = ({ data }) => {
  const alerts = data.filter(d => d.action.includes("brake"));

  return (
    <div className="card">
      <h2 >🚨 Alerts</h2>

      {alerts.map((d, i) => (
        <p key={i} className="alert">
          Brake at step {d.step}
        </p>
      ))}
    </div>
  );
};

export default Alerts;