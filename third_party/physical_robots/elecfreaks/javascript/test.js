let MBIT_UART_SERVICE = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'.toLowerCase(); //to send TO the microbit
let MBIT_UART_RX_CHARACTERISTIC = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'.toLowerCase(); //to send TO the microbit
let MBIT_UART_TX_CHARACTERISTIC = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'.toLowerCase(); //to receive data FROM the microbit
let connectButton = document.getElementById("connectButton");
let helloButton = document.getElementById("helloButton");
let backward_button = document.getElementById("backward");
let left_button = document.getElementById("leftButton");
let right_button = document.getElementById("rightButton");
let logRegion = document.getElementById("log");
let logCount = 0;

// Setup WebSocket connection
const ws = new WebSocket('ws://127.0.0.1:9052');
ws.onopen = function() {
  console.log('WebSocket connection established');

  ws.onmessage = function(event) {
    console.log('Received message from server test:', event.data);
    ourMicrobitUART.send(event.data);
  }
}
function appendToLog(moreText) {
    logCount += 1;
    logRegion.innerHTML += `${logCount}:  ${moreText}  <br>`;
}
let ourMicrobitUART;
let bluetoothSearchOptions = {
    filters: [{
            namePrefix: "BBC micro:bit",
        }],
    optionalServices: [MBIT_UART_SERVICE]
};
class MicroBitUART {
    constructor(rxCharacteristic, txCharacteristic) {
        this.messageSubscribers = [];
        this.rxCharacteristic = rxCharacteristic;
        this.txCharacteristic = txCharacteristic;
        this.decoder = new TextDecoder();
        this.txCharacteristic.startNotifications().then(characteristic => {
            characteristic.addEventListener('characteristicvaluechanged', ev => {
                let value = (event.target).value;
                let valueAsString = new TextDecoder().decode(value);
                this.handleNewMessage(valueAsString);
            });
        });
    }
    subscribeToMessages(receiver) {
        this.messageSubscribers.push(receiver);
    }
    handleNewMessage(message) {
        this.messageSubscribers.forEach(subscriber => {
            subscriber(message);
        });
    }

    send(key, value) {
        let kvstring = `${key}^${value}#`;
        let encoder = new TextEncoder('utf-8');
        let encoded = encoder.encode(kvstring);
        this.rxCharacteristic.writeValue(encoded);
        appendToLog("Sent >>>> " + kvstring);
    }
}
function connectClicked(e) {
    navigator.bluetooth.requestDevice(bluetoothSearchOptions).then(device => {
        appendToLog(`Found:  ${device.name}`);
        return device.gatt.connect();
    }).then(server => {
        appendToLog("...connected!");
        return server.getPrimaryService(MBIT_UART_SERVICE);
    }).then(service => {
        return Promise.all([service.getCharacteristic(MBIT_UART_RX_CHARACTERISTIC),
            service.getCharacteristic(MBIT_UART_TX_CHARACTERISTIC)]);
    }).then(rxandtx => {
        let rx;
        let tx;
        [rx, tx] = rxandtx;
        ourMicrobitUART = new MicroBitUART(rx, tx);
        appendToLog("Made a UART!!");
        startReadingFromUART(ourMicrobitUART);
    }).catch(error => {
        console.log(error);
    });
}
function startReadingFromUART(mbit) {
    mbit.subscribeToMessages((data_from_microbit) => {
//        appendToLog("Read <<<< " + data_from_microbit);
        ws.send(data_from_microbit);
    });
}
function helloClicked(e) {
    ourMicrobitUART.send("f#");
}
function backward(e) {
    ourMicrobitUART.send("b#");
}
function left_function(e) {
    ourMicrobitUART.send("l#");
}
function right_function(e) {
    ourMicrobitUART.send("r#");
}
function sayHelloBack(message) {
    ourMicrobitUART.send("hello", "response");
}
connectButton.onclick = connectClicked;
helloButton.onclick = helloClicked;
backward_button.onclick = backward;
left_button.onclick = left_function;
right_button.onclick = right_function;
