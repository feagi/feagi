var microBit;

var iconLeft = [
  ['0', '0', '0', '0', '0'],
  ['0', '1', '0', '1', '0'],
  ['0', '0', '0', '0', '0'],
  ['1', '0', '0', '0', '1'],
  ['0', '1', '1', '1', '0']
]

var iconRight = [
  ['0', '0', '0', '0', '0'],
  ['0', '1', '0', '1', '0'],
  ['0', '0', '0', '0', '0'],
  ['0', '1', '1', '1', '0'],
  ['1', '0', '0', '0', '1']
]

microBit=new uBit();

microBit.onConnect(function(){
  console.log("connected");

  document.getElementById("connected").innerHTML="true";
  document.getElementById("properties").classList.toggle('inactive');



microBit.setButtonACallback(function() {
  try {
    microBit.send("f");
  } catch (error) {
    console.log("An error occurred while sending data to microBit:", error);
  }
  console.log("buttonA pressed");
});



  microBit.setButtonBCallback(function(){
    console.log("buttonB pressed");
  });
});

microBit.onDisconnect(function(){
  console.log("disconnected");
  document.getElementById("connected").innerHTML="false";
});

function searchDevice(){
  microBit.searchDevice();
}

microBit.onBleNotify(function(){
  document.getElementById("buttonA").innerHTML=microBit.getButtonA();
  document.getElementById("buttonB").innerHTML=microBit.getButtonB();

  document.getElementById("acc_X").innerHTML=microBit.getAccelerometer().x;
  document.getElementById("acc_Y").innerHTML=microBit.getAccelerometer().y;
  document.getElementById("acc_Z").innerHTML=microBit.getAccelerometer().z;

  document.getElementById("temp").innerHTML=microBit.getTemperature();
  document.getElementById("bearing").innerHTML=microBit.getBearing();
})


var ledMatrix = [
  ['0', '0', '0', '0', '0'],
  ['0', '0', '0', '0', '0'],
  ['0', '0', '0', '0', '0'],
  ['0', '0', '0', '0', '0'],
  ['0', '0', '0', '0', '0']
]

function updatePixel(x,y,value){
  if (value){
    ledMatrix[x][y]=1;
  }else{
    ledMatrix[x][y]=0;
  }
  microBit.writeMatrixIcon(ledMatrix);
}

function updateText(){
  text=document.getElementById("newText").value;
  console.log(text);
  microBit.writeMatrixText(text);
}
