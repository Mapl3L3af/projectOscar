const express = require('express');
const fs = require('fs');
const path = require('path');
const app = express();
const PORT = 3000;

app.use(express.json());

app.post('/save-address', (req, res) => {
  const address = req.body.address;
  const filePath = path.join(__dirname, 'algoIn.txt');

  fs.appendFile(filePath, address + '\n', (err) => {
    if (err) {
      console.error("Error writing to file:", err);
      return res.status(500).send("Error saving address.");
    }
    res.send("Address saved successfully!");
  });
});

app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
