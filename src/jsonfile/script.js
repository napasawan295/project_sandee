document.addEventListener("DOMContentLoaded", function() {
    const micButton = document.getElementById("micButton");

    // Check if the browser supports getUserMedia
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        // Request microphone access
        navigator.mediaDevices.getUserMedia({ audio: true })
            .then(stream => {
                console.log("Microphone access granted");

                // Create an audio context and media stream source for further processing
                const audioContext = new (window.AudioContext || window.webkitAudioContext)();
                const source = audioContext.createMediaStreamSource(stream);

                // Start audio processing here...

                // Enable the mic button to indicate that recording is active
                micButton.textContent = "Recording...";
                micButton.disabled = true;
            })
            .catch(error => {
                console.error("Microphone access denied", error);
                micButton.textContent = "Microphone access denied";
            });
    } else {
        console.error("getUserMedia not supported in this browser");
        micButton.textContent = "getUserMedia not supported";
    }
});

// Socket.io connection
const socket = io();

// Create a SpeechRecognition object
const recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
recognition.continuous = true;
recognition.interimResults = true;

// Variable to store the recognized sentence
let currentSentence = '';
// Variable to store the latest recognized sentences
let humantext = '';

// Start recognition
recognition.start();

recognition.onstart = () => {
    console.log('Speech recognition started');
};

let timeout;
// Process recognized speech
recognition.onresult = async (event) => {
    const transcript = event.results[event.results.length - 1][0].transcript.trim();
    console.log('Recognized:', transcript);

    // Update current sentence
    currentSentence = transcript;

    // Send the sentences to app.js when speech stops
    clearTimeout(timeout);
    timeout = setTimeout(async () => {
        await sendSentences();

        //recognition.stop();
        recognition.start();
    }, 500); // Adjust the delay as needed
};

// Error handling
recognition.onerror = (event) => {
    console.error('Speech recognition error:', event.error);

    switch (event.error) {
        case 'no-speech':
            console.log('No speech detected. Try again.');
            break;
        case 'audio-capture':
            console.log('No microphone was found. Ensure that a microphone is installed.');
            break;
        case 'not-allowed':
            console.log('Permission to use microphone is denied.');
            break;
        default:
            console.log('An unknown error occurred: ', event.error);
            break;
    }
};

// Function to send sentences to app.js and reset the variable
async function sendSentences() {
    if (currentSentence !== '') {
        humantext = currentSentence;
        // Send the sentence to app.js via socket.io
        await socket.emit('speech', humantext);
        // Clear the current sentence for new input
        currentSentence = '';
    }
}

socket.on('botText', (bottext) => {
    const audio = new Audio();
    tts(bottext);
    console.log("TTS processed");

    async function tts(bottext) {
        const form = new FormData();
        form.append("text", bottext);

        try {
            const response = await fetch('http://boonchuai-eks-ingress-799182153.ap-southeast-1.elb.amazonaws.com/tts', {
                method: 'POST',
                body: form,
                headers: {
                    'x-access-token': 'YOUR_ACCESS_TOKEN' // Replace with your access token
                }
            });

            if (response.ok) {
                const wavBlob = await response.blob();
                const wavUrl = URL.createObjectURL(wavBlob);
                playAudio(wavUrl);
            } else {
                console.error('Failed to call the TTS API:', response.status, response.statusText);
            }
        } catch (error) {
            console.error('Error calling the TTS API:', error);
        }
    }

    async function playAudio(wavUrl) {
        if (audio.src) {
            URL.revokeObjectURL(audio.src);
            audio.src = '';
            audio.pause();
            audio.currentTime = 0;
        }
    
        audio.src = wavUrl;
        try {
            await audio.play();
            // เปลี่ยนสีของปุ่มเป็นสีเขียวจางเมื่อเล่นเสียง
            toggleSoundButton.style.backgroundColor = 'lightgreen';
        } catch (error) {
            console.error('Failed to play audio:', error);
            // เปลี่ยนสีของปุ่มเป็นสีแดงจางเมื่อเกิดข้อผิดพลาด
            toggleSoundButton.style.backgroundColor = 'lightcoral';
        }
    }
    
});

// เลือกปุ่ม
const toggleSoundButton = document.getElementById('toggleSoundButton');

// เพิ่มเหตุการณ์การคลิกปุ่ม
toggleSoundButton.addEventListener('click', toggleSound);

// ฟังก์ชันที่ใช้ในการเปิด-ปิดเสียง
function toggleSound() {
    // ตรวจสอบว่าเสียงกำลังเล่นหรือไม่
    if (audio.paused) {
        // เล่นเสียง
        audio.play();
        // เปลี่ยนสีของปุ่มเป็นสีเขียวจาง
        toggleSoundButton.style.backgroundColor = 'lightgreen';
    } else {
        // หยุดเล่นเสียง
        audio.pause();
        // เปลี่ยนสีของปุ่มเป็นสีแดงจาง
        toggleSoundButton.style.backgroundColor = 'lightcoral';
    }
} 
