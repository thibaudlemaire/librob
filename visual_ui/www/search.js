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

var UI_feedback_listener = new ROSLIB.Topic({
    ros : rbServer,
    name : '/UI',
    messageType : 'librarian_msgs/UI_feedback'
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


// Publisher function triggered by the search button
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

    constructTable();
}


// Listened callback triggered by a ROS message
UI_feedback_listener.subscribe(function(message) {

    var books = JSON.parse(message.data).books;
    var tableContent = '';

    for (var r = 0; r < books.length; r++){
        tableContent += '<tr>';
        book = books[r]
        for (var prop in book) {
            if (book.hasOwnProperty(prop)) {
                tableContent += '<td>' + book[prop] + '</td>';
            }
        }
        tableContent += '</tr>';
    }

    var table = document.getElementById('searchResults');
    table.innerHTML +=  tableContent;
    
    UI_feedback_listener.unsubscribe();
});




testData = JSON.stringify({ books: [
       {    title: 'Machine learning',
            author: 'Mitchell, Tom M. (Tom Michael)',
            code: '006.31 MIT',
            floor: 1,
            available: true },
        {   title: 'Red seas under red skies',
            author: 'Lynch, Scott',
            code: '800 LYN',
            floor: 5,
            available: true }
       ]
     });


function constructTable() {

    var books = JSON.parse(testData).books;
    var tableContent = '';

    for (var r = 0; r < books.length; r++){
        tableContent += '<tr>';
        book = books[r]
        for (var prop in book) {
            if (book.hasOwnProperty(prop)) {
                tableContent += '<td>' + book[prop] + '</td>';
            }
        }
        tableContent += '</tr>';
    }

    var table = document.getElementById('searchResults');
    table.innerHTML +=  tableContent;

}
