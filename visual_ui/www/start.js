const SPEECH_TRIGGER = 1;
const SEARCH_REQUEST = 10;



// This function connects to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
 });

 // These lines create a topic object as defined by roslibjs
var UI_topic = new ROSLIB.Topic({
    ros : rbServer,
    name : '/ui_command',
    messageType : 'librarian_msgs/UI'
});

 // This function is called upon the rosbridge connection event
 rbServer.on('connection', function() {
    console.log('Connected to websocket server.');
 });

// This function is called when there is an error attempting to connect to rosbridge
rbServer.on('error', function(error) {
    console.log('Error connecting to websocket server.');
});

// This function is called when the connection to rosbridge is closed
rbServer.on('close', function() {
    console.log('Connection to websocket server closed.');
 });




function startButton_callback() {

    var UI_msg = new ROSLIB.Message({
        type: SPEECH_TRIGGER
    });


    // Publish the message
    UI_topic.publish(UI_msg);

    window.location.href = 'search.html';
}