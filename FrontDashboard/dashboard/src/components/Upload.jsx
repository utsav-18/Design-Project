import Papa from "papaparse";

const Upload = ({ setData }) => {
  const handleFile = (e) => {
    const file = e.target.files[0];

    Papa.parse(file, {
      header: true,
      skipEmptyLines: true,
      complete: (results) => {
        setData(results.data);
      },
    });
  };

  return (
    <div>
      <input type="file" accept=".csv" onChange={handleFile} />
    </div>
  );
};

export default Upload;