const modal = document.getElementById("overrideModal");
const modalHeader = document.getElementById("modalHeader");
var currentModal = 1;
// slider
var slider = document.getElementById("overrideRange");
var overrideDuration = document.getElementById("overrideDuration");

function openModal(button) {
  modal.style.display = "block";
  var header = button.getAttribute('data-header');
  modalHeader.textContent = header;
  if (header === "Water Pump 1 Override"){
  	currentModal = 1;
  } else if (header === "Water Pump 2 Override"){
  	currentModal = 2;
  } else {
  	currentModal = 3;
  }
}

function closeModal() {
  modal.style.display = "none";
}
// When the user clicks anywhere outside of the modal, close it
window.onclick = function(event) {
  if (event.target == modal) {
    modal.style.display = "none";
  }
}
slider.oninput = function() {
  if(this.value == "65"){
	overrideDuration.innerHTML = "Permanent";
  } else {
  	overrideDuration.innerHTML = this.value + " min";
  }
}
// set pump override
function setOverride(button) {
  //output = relay pin number
  //state = on/off
  //time = override time in minutes (65 = permanent)
  var output;  // output relay pin to command
  if (currentModal == 1){
  	output = "22";
  } else if(currentModal == 2){
  	output = "21";
  } else {
  	output = "19";
  }
  var sliderValue = slider.value;
  var xhr = new XMLHttpRequest();
  var dataValue = button.getAttribute("data-value");
  xhr.open("GET", "/override?output=" + output + "&state=" + dataValue + "&time=" + sliderValue, true); 
  xhr.send();
  // update status
  var command = (dataValue == "1") ? "On" : "Off"
  var timeLeft = (sliderValue == "65") ? "Permanent)" : sliderValue + " min)"
  command += " (Override " + timeLeft;
  if (currentModal == 1){
  	document.getElementById("pump1Command").innerHTML = command;
  } else if(currentModal == 2){
  	document.getElementById("pump2Command").innerHTML = command;
  } else {
  	document.getElementById("airPumpCommand").innerHTML = command;
  }
  closeModal();
}
function setAuto(button){
  var xhr = new XMLHttpRequest();
  var relay = button.getAttribute("data-relay");
  xhr.open("GET", "/auto?output=" + relay, true); 
  xhr.send();
  closeModal();
}
function changeWaterLevelTextColor(){
  var waterLevel = document.getElementById("waterLevel").innerHTML;
  if (waterLevel == "Low"){
    // change document.getElementById("waterLevel") text color to red
    document.getElementById("waterLevel").style.color = "red";
  } else if (waterLevel == "Medium"){
    document.getElementById("waterLevel").style.color = "orange";
  } else {
    document.getElementById("waterLevel").style.color = "green";
  }
}

if (!!window.EventSource) {
  var source = new EventSource('/events');
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);
  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);
  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);
  source.addEventListener('temperature', function(e) {
    console.log("temperature", e.data);
    document.getElementById("temp").innerHTML = e.data;
  }, false);
  source.addEventListener('humidity', function(e) {
    console.log("humidity", e.data);
    document.getElementById("hum").innerHTML = e.data;
  }, false);
  source.addEventListener('heatIndex', function(e) {
    console.log("heatIndex", e.data);
    document.getElementById("heat").innerHTML = e.data;
  }, false);
  source.addEventListener('pump1Status', function(e) {
    console.log("pump1Status", e.data);
    document.getElementById("pump1Status").innerHTML = e.data;
  }, false);
  source.addEventListener('pump2Status', function(e) {
    console.log("pump2Status", e.data);
    document.getElementById("pump2Status").innerHTML = e.data;
  }, false);
  source.addEventListener('airPumpStatus', function(e) {
    console.log("airPumpStatus", e.data);
    document.getElementById("airPumpStatus").innerHTML = e.data;
  }, false);
  source.addEventListener('pump1Command', function(e) {
    console.log("pump1Command", e.data);
    document.getElementById("pump1Command").innerHTML = e.data;
  }, false);
  source.addEventListener('pump2Command', function(e) {
    console.log("pump2Command", e.data);
    document.getElementById("pump2Command").innerHTML = e.data;
  }, false);
  source.addEventListener('airPumpCommand', function(e) {
    console.log("airPumpCommand", e.data);
    document.getElementById("airPumpCommand").innerHTML = e.data;
  }, false);
  source.addEventListener('waterLevel', function(e) {
    console.log("waterLevel", e.data);
    document.getElementById("waterLevel").innerHTML = e.data;
    changeWaterLevelTextColor();
  }, false);
}
