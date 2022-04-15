// Complete project details: https://randomnerdtutorials.com/esp32-web-server-websocket-sliders/

const gateway = `ws://${window.location.hostname}/ws`;
let websocket;

const onOpen = (event) => {
    console.log('Connection opened');
}
const onClose = (event) => {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}
const onMessage = (event) => {
    const data = JSON.parse(event.data);
    for (const [key, value] of Object.entries(data)) {
        console.log(`${key}: ${value}`);
        document.getElementById("data-" + key).innerHTML = value;
      }
}
const initWebSocket = () => {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

window.addEventListener('load', () => {
    initWebSocket();
    document.querySelectorAll('input[type=range]').forEach((element) => {
        element.addEventListener('input', (event) => {
            var sliderNumber = element.id.charAt(element.id.length - 1);
            var sliderValue = document.getElementById(element.id).value;
            document.getElementById("sliderValue" + sliderNumber).innerHTML = sliderValue;
            console.log(sliderValue);
            websocket.send(sliderNumber + "s" + sliderValue.toString());
        });
    });
    document.querySelectorAll('input[type=number]').forEach((element) => {
        element.addEventListener('input', (event) => {
            console.log(element.id + element.value)
            websocket.send(element.id + element.value);
        });
    })
})
