const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const path = require('path');
const fs = require('fs');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// Serve static files
app.use(express.static(path.join(__dirname, 'public')));
app.use(express.static('/home/aew/sandee/src/jsonfile'));

// Serve HTML, CSS, and JS files
app.get('/', function (_req, res) {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.get('/styles.css', function (_req, res) {
    res.sendFile(path.join(__dirname, 'styles.css'));
});

app.get('/script.js', function (_req, res) {
    res.sendFile(path.join(__dirname, 'script.js'));
});

io.on('connection', (socket) => {
    console.log('A user connected');

    socket.on('speech', async (humantext) => {
        console.log('Received transcript:', humantext);
        await nlp(humantext);
    });

    socket.on('disconnect', () => {
        console.log('A user disconnected');
    });
});

async function nlp(text) {
    let humantext = text;
    let bottext = '';

    if (humantext.includes("hello")) {
        bottext = 'สวัสดีค่ะ';
    } else if (humantext.includes("what's your name")) {
        bottext = 'ฉันแสนดีค่ะ';
    } else if (humantext.includes("you so cute")) {
        bottext = 'ขอบคุณค่ะ';
    } else if (humantext.includes("thank you")) {
        bottext = 'ยินดีค่ะ';
    } else if (humantext.includes("0")) {
        bottext = 'Home';
    } else if (humantext.includes("1")) {
        bottext = 'Festo';
    } else if (humantext.includes("2")) {
        bottext = 'Mitsubishi';
    } else if (humantext.includes("3")) {
        bottext = 'Kuka';
    } else if (humantext.includes("4")) {
        bottext = 'ABB1';
    } else if (humantext.includes("5")) {
        bottext = 'ABB2';
    } else if (humantext.includes("6")) {
        bottext = 'ABB3';
    }

    // Create JSON object based on conditions
    let jsonObject = {};

    if (humantext.includes("0")) {
        jsonObject["Home"] = {
            "number": 0
        };
    } else if (humantext.includes("1")) {
        jsonObject["Festo"] = {
            "number": 1
        };
    } else if (humantext.includes("2")) {
        jsonObject["Mitsubishi"] = {
            "number": 2
        };
    } else if (humantext.includes("3")) {
        jsonObject["Kuka"] = {
            "number": 3
        };
    } else if (humantext.includes("4")) {
        jsonObject["ABB 1"] = {
            "number": 4
        };
    } else if (humantext.includes("5")) {
        jsonObject["ABB 2"] = {
            "number": 5
        };
    } else if (humantext.includes("6")) {
        jsonObject["ABB 3"] = {
            "number": 6
        };
    } else {
        jsonObject[humantext] = {
            "number": null
        };
    }

    // Convert JSON object to string
    const jsonString = JSON.stringify(jsonObject, null, 2); // 2-space indentation

    // Write to a JSON file
    const jsonFilePath = path.join('/home/aew/sandee/src/jsonfile', 'control.json'); // Correct file path
    fs.writeFileSync(jsonFilePath, jsonString);

    // Send bottext to client via socket.io
    io.emit('botText', bottext);
    console.log('Bot Answer:', bottext);

    // Return bottext
    return bottext;
}

function clearJsonFile() {
    const jsonFilePath = path.join('/home/aew/sandee/src/jsonfile', 'control.json');
    const emptyJson = JSON.stringify({}, null, 2); // Create an empty JSON object

    try {
        fs.writeFileSync(jsonFilePath, emptyJson);
        console.log('control.json file cleared');
    } catch (err) {
        console.error('Error clearing control.json file:', err);
    }
}

const PORT = process.env.PORT || 5050;

// Clear the JSON file before starting the server
clearJsonFile();

server.listen(PORT, () => {
    console.log(`Server running on port ${PORT}`);
});

