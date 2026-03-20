const SummaryCards = ({ data }) => {
  const totalBrakes = data.filter(d => d.action.includes("brake")).length;

  const minGap = Math.min(...data.map(d => d.gap));

  const risky = data.filter(d => d.gap < d.safe).length;

  return (
        <div className="summary">

    <div className="card">
        <h3>Total Brakes</h3>
        <p className="red">{totalBrakes}</p>
    </div>

    <div className="card">
        <h3>Minimum Gap</h3>
        <p className="yellow">{minGap}</p>
    </div>

    <div className="card">
        <h3>Risk Events</h3>
        <p className="pink">{risky}</p>
    </div>

    </div>
  );
};

export default SummaryCards;