

const SPEECH_TRIGGER = 1;


// This function connects to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
 });

 // This function is called upon the rosbridge connection event
 rbServer.on('connection', function() {
     // Write appropriate message to #feedback div when successfully connected to rosbridge
     var fbDiv = document.getElementById('feedback');
     fbDiv.innerHTML += "<p>Connected to websocket server.</p>";
 });

// This function is called when there is an error attempting to connect to rosbridge
rbServer.on('error', function(error) {
    // Write appropriate message to #feedback div upon error when attempting to connect to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Error connecting to websocket server.</p>";
});

// This function is called when the connection to rosbridge is closed
rbServer.on('close', function() {
    // Write appropriate message to #feedback div upon closing connection to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Connection to websocket server closed.</p>";
 });

// These lines create a topic object as defined by roslibjs
var UI_topic = new ROSLIB.Topic({
    ros : rbServer,
    name : '/UI',
    messageType : 'librarian_msgs/UI'
});

// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
var UI_msg = new ROSLIB.Message({
    type: SPEECH_TRIGGER
});

/* This function:
 - publishes the message to the UI_topic topic.
 */
function pubMessage() {


    // Publish the message 
    UI_topic.publish(UI_msg);
}