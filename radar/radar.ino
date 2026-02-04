#include <WiFi.h>
#include <WebServer.h>
#include <Servo.h>

#define SERVO_PIN 18
#define RADAR_OUT_PIN 0

Servo radarServo;
WebServer server(80);

int angle = 0;
bool forward = true;

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>ESP32 Radar</title>
<style>
body { background:black; color:lime; text-align:center; }
canvas { background:black; }
</style>
</head>
<body>
<h2>RADAR ONLINE</h2>
<canvas id="radar" width="300" height="150"></canvas>
<script>
const c = document.getElementById("radar");
const ctx = c.getContext("2d");
const r = 140;

function draw(angle, detected) {
  ctx.clearRect(0,0,300,150);
  ctx.strokeStyle="lime";
  ctx.beginPath();
  ctx.arc(150,150,r,Math.PI,2*Math.PI);
  ctx.stroke();

  let a = angle * Math.PI / 180;
  ctx.beginPath();
  ctx.moveTo(150,150);
  ctx.lineTo(150 + r*Math.cos(Math.PI+a), 150 + r*Math.sin(Math.PI+a));
  ctx.stroke();

  if(detected){
    ctx.fillStyle="red";
    ctx.beginPath();
    ctx.arc(
      150 + (r-20)*Math.cos(Math.PI+a),
      150 + (r-20)*Math.sin(Math.PI+a),
      6,0,2*Math.PI);
    ctx.fill();
  }
}

setInterval(async ()=>{
  let r = await fetch("/data");
  let j = await r.json();
  draw(j.angle, j.detected);
},200);
</script>
</body>
</html>
)rawliteral";

void handleRoot(){
  server.send_P(200,"text/html",PAGE);
}

void handleData(){
  String json = "{";
  json += "\"angle\":" + String(angle) + ",";
  json += "\"detected\":" + String(digitalRead(RADAR_OUT_PIN));
  json += "}";
  server.send(200,"application/json",json);
}

void setup(){
  pinMode(RADAR_OUT_PIN, INPUT);
  radarServo.attach(SERVO_PIN);

  WiFi.softAP("ESP32-RADAR","12345678");

  server.on("/",handleRoot);
  server.on("/data",handleData);
  server.begin();
}

void loop(){
  server.handleClient();

  radarServo.write(angle);
  delay(20);

  if(forward){
    angle++;
    if(angle>=180) forward=false;
  }else{
    angle--;
    if(angle<=0) forward=true;
  }
}
