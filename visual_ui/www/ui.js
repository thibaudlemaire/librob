

const SPEECH_TRIGGER = 1;
const SEARCH_REQUEST = 10;




// This function connects to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
 });

 // These lines create a topic object as defined by roslibjs
var UI_topic = new ROSLIB.Topic({
    ros : rbServer,
    name : '/UI',
    messageType : 'librarian_msgs/UI'
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




/* This function:
 - publishes the message to the UI_topic topic.
 */
function startButton_callback() {


    var UI_msg = new ROSLIB.Message({
        type: SPEECH_TRIGGER
    });


    // Publish the message 
    UI_topic.publish(UI_msg);

    var elem = document.getElementById("startButton");
    elem.innerText="Insert book name" ;
    var x=document.getElementById("textbox");
    x.style.display= "block";
}


function searchButton_callback()
{

    var bookTitle = document.getElementById("textInput").value;

    var UI_msg = new ROSLIB.Message({
        type: SEARCH_REQUEST,
        payload: JSON.stringify({
            "request": bookTitle
        }) 
    });

    UI_topic.publish(UI_msg);

}