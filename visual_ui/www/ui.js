// UI
const SPEECH_TRIGGER = 1;
const SEARCH_REQUEST = 10;
const BOOK_CHOSEN = 20;
// UI_feedback
const SEARCH_RESPONSE = 1;
const LOADING = 2;
const COMMUNICATION = 3;
const LISTENING = 7;

// Connect to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    //url : 'ws://' + window.location.hostname + ':9090'
    url :'ws://localhost:9090'
 });

 // Create a UI topic object
var UI = new ROSLIB.Topic({
    ros : rbServer,
    name : '/ui_command',
    messageType : 'librarian_msgs/UI'
});

 // Create a UI_feedback topic object
var UI_feedback = new ROSLIB.Topic({
    ros : rbServer,
    name : '/ui_feedback',
    messageType : 'librarian_msgs/UI_feedback'
    });

 // Upon the rosbridge connection event
 rbServer.on('connection', function() {
    console.log('Connected to websocket server.');
 });

// Error attempting to connect to rosbridge
rbServer.on('error', function(error) {
    console.log('Error connecting to websocket server.');
});

// Connection to rosbridge is closed
rbServer.on('close', function() {
    console.log('Connection to websocket server closed.');
 });



// Subscriber of the UI_feedback topic
UI_feedback.subscribe(function(message) {

    var uiFeedbackText = document.getElementById('uiFeedback')
    var communicationFeedbackText = document.getElementById('communicationFeedback')

    if (message.type == LISTENING) {
        uiFeedbackText.innerHTML = 'Listening';
    }

    else if (message.type == LOADING) {
        uiFeedbackText.innerHTML = 'Loading';
    }

    else if (message.type == SEARCH_RESPONSE) {
        uiFeedbackText.innerHTML = 'Search Response';
        $('#result_modal').modal('show');
        var books = JSON.parse(message.payload).books;
        createTable(books);
    }

    else if (message.type == COMMUNICATION) {
        var msg = JSON.parse(message.payload).message;
        communicationFeedbackText.innerHTML = msg;
    }

});



// Callback when speech icon is clicked
 function speechButtonCallback() {
     
    selector = document.getElementById('selector');
    language = selector.options[selector.selectedIndex].value;

    var UI_msg = new ROSLIB.Message({
        type: SPEECH_TRIGGER,
        payload: JSON.stringify({
            'language': language
        })
    });

    // Publish the message
    UI.publish(UI_msg);

}



// Publisher: publishes on UI topic a SEARCH_REQUEST message
function searchButtonCallback()
{
    var bookTitle = document.getElementById("textInput").value;

    var UI_msg = new ROSLIB.Message({
        type: SEARCH_REQUEST,
        payload: JSON.stringify({
            "request": bookTitle
        }) 
    });

    UI.publish(UI_msg);

}




// Visual feedback: populate a table with books from the SEARCH_RESPONSE message
function createTable(books) {
    var table = document.getElementById('searchResults');
    table.style.visibility = "visible";
    var tbody = document.getElementsByTagName('tbody')[0];
    // Iterate over list of books
    for (var r = 0; r < books.length; r++){
        book = books[r];
        // Insert a row <tr></tr> for each book
        var row = tbody.insertRow(-1);

        // Iterate over keys of each book
        Object.keys(book).forEach(function(k){
            // Insert a cell <td></td> for each key
            var cell = row.insertCell();
            cell.class = "align-middle";
            cell.class = "book-" + k;
            cell.innerHTML = book[k];
        });
        // Add one last cell which an icon button
        var cell = row.insertCell();
        cell.class = "align-middle";
        cell.innerHTML = "<button class='btn btn-md btn-primary'><i class='fas fa-walking'></i></button>";
    
    row.onclick =  sendBookCode;
    //$("table tbody tr button").on('click', function(e){
        //alert($(this).closest('td').parent()[0].sectionRowIndex);
    // });​
    }
}



// Publisher: publishes on UI topic a BOOK_CHOSEN message
function sendBookCode() {

    var UI_msg = new ROSLIB.Message({
        type: BOOK_CHOSEN,
        payload: JSON.stringify({
            "chosen_code": this.childNodes[2].innerHTML
        }) 
    });

    UI.publish(UI_msg);

}