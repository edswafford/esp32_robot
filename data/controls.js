const controllerStates = {
    DISCONNECTED: 0,
    RUNNING: 1,
    STOPPED: 2
};

var ws = null;
var connected = false;
let controllerState = controllerStates.DISCONNECTED;

const speedRange = document.querySelectorAll(".speed-range-wrap");
const steeringRange = document.querySelectorAll(".steering-range-wrap");

speedRange.forEach(wrap => {
    const range = wrap.querySelector(".range");
    const bubble = wrap.querySelector(".bubble");

    range.addEventListener("input", () => {
        setBubble(range, bubble);
        sendSpeed(range.value);
    });
    setBubble(range, bubble);
});

steeringRange.forEach(wrap => {
    const range = wrap.querySelector(".range");
    const bubble = wrap.querySelector(".bubble");

    range.addEventListener("input", () => {
        setBubble(range, bubble);
        sendSteering(range.value);
    });
    setBubble(range, bubble);
});

function setBubble(range, bubble) {
    const val = range.value;
    const min = range.min ? range.min : 0;
    const max = range.max ? range.max : 100;
    const newVal = Number(((val - min) * 100) / (max - min));
    bubble.innerHTML = val;

    // Sorta magic numbers based on size of the native UI thumb
    bubble.style.left = `calc(${newVal}% + (${8 - newVal * 0.15}px))`;
}


function EmergencyStop() {

    if (controllerState === controllerStates.RUNNING) {
    	  // reset controls BEFORE changing states
    	  resetSpeed();
        resetSteering();
        controllerState = controllerStates.STOPPED;
        var stopButton = document.getElementById("stopButton");
        stopButton.style.backgroundColor = "Green";
        stopButton.innerHTML = "RUN";
    } else if (controllerState === controllerStates.STOPPED) {
        controllerState = controllerStates.RUNNING;
        var stopButton = document.getElementById("stopButton");
        stopButton.style.backgroundColor = "Red";
        stopButton.innerHTML = "EMERGENCY STOP";
        // reset controls AFTER changing states
        resetSpeed();
        resetSteering();
    }
}

function disconnect() {

}
function OpenWebsocket() {
    var ipAddress = "ws://" + document.getElementById("esp32-ip").innerHTML + "/controller";
    ws = new WebSocket(ipAddress);
    // ws = new WebSocket("ws://192.168.4.1/controller");

    ws.onopen = function () {
        connected = true;
        controllerState = controllerStates.RUNNING;
        document.getElementById("connectButton").disabled = true;
        document.getElementById("disconnectButton").disabled = false;
        var stateButton = document.getElementById("stopButton");
        stateButton.disabled = false;
        stateButton.style.backgroundColor = "Red";
        stateButton.innerHTML = "EMERGENCY STOP";
        resetSpeed();
        resetSteering();
    };

    ws.onclose = function () {
        connected = false;
        controllerState = controllerStates.DISCONNECTED;
        var stopButton = document.getElementById("stopButton");
        stopButton.style.backgroundColor = "inactivecaption";
        stopButton.innerHTML = "STOPPED";
        resetSpeed();
        resetSteering();

        document.getElementById("connectButton").disabled = false;
        document.getElementById("disconnectButton").disabled = true;

        alert("Connection closed");
    };
}

function CloseWebsocket() {
    if (ws) {
        ws.close();
    }
}

function resetSpeed() {
    document.getElementById("speed-input-range").value = 0;
    speedRange.forEach(wrap => {
        const range = wrap.querySelector(".range");
        const bubble = wrap.querySelector(".bubble");
        setBubble(range, bubble);
        sendSpeed(0);
    });
}

function resetSteering() {
    document.getElementById("steering-input-range").value = 0;
    steeringRange.forEach(wrap => {
        const range = wrap.querySelector(".range");
        const bubble = wrap.querySelector(".bubble");
        setBubble(range, bubble);
        sendSteering(0);
    });
}
function sendSpeed(val) {
    if (ws !== null && controllerState === controllerStates.RUNNING) {
        var y = "y";
        ws.send(y.concat(val));
    }
}
function sendSteering(val) {
    if (ws !== null && controllerState === controllerStates.RUNNING) {
        var x = "x";
        ws.send(x.concat(val));
    }
}
