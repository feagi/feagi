const video = document.createElement('video');
const canvas = document.createElement('canvas');
const ctx = canvas.getContext('2d');

// Setup WebSocket connection
const ws = new WebSocket('ws://127.0.0.1:9051');
ws.onopen = function() {
  console.log('WebSocket connection established');
}

// Start capturing video
navigator.mediaDevices.getUserMedia({ video: true }).then(function(stream) {
  video.srcObject = stream;
  video.play();
  requestAnimationFrame(processFrame);
});

function processFrame() {
  // Draw video frame onto canvas
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;
  ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

  // Get raw RGB data from canvas
  const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
  const rgbArray = imageData.data;

  // Send raw RGB data to WebSocket
  ws.send(rgbArray);

  // Request next frame
  requestAnimationFrame(processFrame);
}
