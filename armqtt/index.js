const express = require("express");
const app = express();
const server = app.listen(3000);
var path = require("path");
const io = require("socket.io")(server);
var mqtt = require("mqtt");
// var client = mqtt.connect("mqtt://127.0.0.1");
var client = mqtt.connect("mqtt://127.0.0.1");

client.on("connect", function() {
    client.subscribe("hello");
});


//expose the local public folder for inluding files js, css etc..
app.use(express.static("public"));

app.get('/', function(req, res) {
    res.sendFile(__dirname + '/index.html');
});

client.on("message", function(topic, message) {
    io.sockets.emit("data", String(message));
    console.log(String(message));
});