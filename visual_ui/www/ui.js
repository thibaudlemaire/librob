// UI
const SPEECH_TRIGGER = 1;
const SEARCH_REQUEST = 10;
const BOOK_CHOSEN = 20;
// UI_feedback
const SEARCH_RESPONSE = 1;
const LOADING = 2;
const COMMUNICATION = 3;
const LISTENING = 7;

$('#speech-bubble').hide();

// Connect to the rosbridge server running on the local computer on port 9090
var rbServer = new ROSLIB.Ros({
    url : 'ws://' + window.location.hostname + ':9090'
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

    if (message.type === LISTENING) {
        if (JSON.parse(message.payload) === true) {
            $('#mic_icon').addClass('blinking');
        }
        else {
            $('#mic_icon').removeClass('blinking');
        }
    }
    else if (message.type === LOADING) {
        if (JSON.parse(message.payload) === true) {
            $('#mic_icon').removeClass('blinking');
            $('#search_modal').modal('hide');
            $('#loading_modal').modal('show');
        }
        else {
            $('#loading_modal').modal('hide');
        }
    }
    else if (message.type === SEARCH_RESPONSE) {
        var books = JSON.parse(message.payload).books;
        if (books.length !== 0) {
            $('#loading_modal').modal('hide');
            $('#search_modal').modal('hide');
            $('#result_modal').modal('show');
            createTable(books);
        }
    }
    else if (message.type === COMMUNICATION) {
        $('#loading_modal').modal('hide');
        var msg = JSON.parse(message.payload).message;
        var bubble = $('#speech-bubble');
        if (bubble.is(':visible')) {
            bubble.stop(true).show().delay(3000 + msg.length * 50).fadeOut();
        } else {
            bubble.fadeIn().delay(3000 + msg.length * 50).fadeOut();
        }
        bubble.text(msg);
        responsiveVoice.speak(msg,"UK English Male", {pitch:9},{volume: 1},{rate: 10});
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

    UI.publish(UI_msg);

}



// Publisher: publishes on UI topic a SEARCH_REQUEST message
function searchButtonCallback()
{
    var textfield = $("#textInput");

    var UI_msg = new ROSLIB.Message({
        type: SEARCH_REQUEST,
        payload: JSON.stringify({
            "request": {
                'author': '',
                'title': textfield.val()
            }
        }) 
    });
    
    UI.publish(UI_msg);
    textfield.val("");
}



// Visual feedback: populate a table with books from the SEARCH_RESPONSE message

function createTable(books) {

    var tbody = document.getElementsByTagName('tbody')[0];
    tbody.innerHTML = "";

    // Iterate over list of books
    for (var r = 0; r < books.length; r++){
        book = books[r];
        // Insert a row <tr></tr> for each book
        var row = tbody.insertRow(-1);

        var cell = row.insertCell();
        cell.setAttribute('class', 'align-middle title');
        cell.innerHTML = book['title'];

        var cell = row.insertCell();
        cell.setAttribute('class', 'align-middle author');
        cell.innerHTML = book['author'];

        var cell = row.insertCell();
        cell.setAttribute('class', 'align-middle code');
        cell.innerHTML = book['code'];

        var cell = row.insertCell();
        cell.setAttribute('class', 'align-middle floor');
        cell.innerHTML = book['floor'];

        // Add one last cell which an icon button
        var cell = row.insertCell();
        cell.setAttribute('class', 'align-middle');
        if (book['available'] === true){
            cell.innerHTML = "<button class='btn btn-md btn-primary'><i class='fas fa-walking'></i></button>";
        }
        else {
            row.classList.add('table-secondary');
            cell.innerHTML = "<button class='btn btn-md btn-primary' disabled><i class='fas fa-walking'></i></button>";
        }
        
    }

    $("table tbody tr td button").on('click', function(e){
        var rowIndex = $(this).closest('td').parent()[0].sectionRowIndex;
        var codes = $('.code');
        var code = codes[rowIndex].innerText;
        sendBookCode(code);
        $('#result_modal').modal('hide');
    })
}


// Publisher: publishes on UI topic a BOOK_CHOSEN message
function sendBookCode(code) {

    var UI_msg = new ROSLIB.Message({
        type: BOOK_CHOSEN,
        payload: JSON.stringify({
            "chosen_code": code
        }) 
    });

    UI.publish(UI_msg);
}
