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
}
