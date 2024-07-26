var app = require("express")();
var express = require("express");
var http = require('http').Server(app);
var bodyParser = require('body-parser');

app.use(express.json({limit: '50mb'}));
app.use(express.urlencoded({limit: '50mb'}));

//app.use(bodyParser.json())
app.use((req, res, next) => {
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.header(
    "Access-Control-Allow-Headers",
    "Origin, X-Requested-With, Content-Type, Accept"
  );
  next();
});
img1Msg = ""

app.post('/img1',function(req,res){
        var msg=req.body.msg;
        //console.log("python: " + msg);
        img1Msg = msg
});
app.get("/img1",function(req,res){
        //console.log(img1Msg)
        res.send(img1Msg)
});

http.listen(5000, function(){
  console.log('listening...');
});
