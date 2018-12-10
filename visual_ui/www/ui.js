// UI
const SPEECH_TRIGGER = 1;
const SEARCH_REQUEST = 10;
const BOOK_CHOSEN = 20;
const LETS_GO = 40;
// UI_feedback
const SEARCH_RESPONSE = 1;
const LOADING = 2;
const COMMUNICATION = 3;
const LISTENING = 7;
const RESET = 8;
const DISPLAY_GO = 9;


var COMMUNICATION_MESSAGES;
$.getJSON( "COMMUNICATION_MESSAGES.json", function( json ) {
    COMMUNICATION_MESSAGES = json;
});

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
        var payload = JSON.parse(message.payload); // keys: 'speak', 'title', 'author', 'type'
        var type = payload.type;
        var title = payload.title;
        var author = payload.author;

        selector = document.getElementById('selector');
        language = selector.options[selector.selectedIndex].value;

        var msg = '';
        if (type === 'FOUND'){
            msg = foundMessage(language, title, author);
        }
        else if (type === 'NOT_FOUND'){
            msg = notFoundMessage(language, title, author);
        }
        else {
            msg = COMMUNICATION_MESSAGES[language][type];
        }
        var bubble = $('#speech-bubble');
        if (bubble.is(':visible') || bubble.is(':animated')) {
            bubble.stop(true);
            bubble.show().delay(5000 + msg.length * 200).fadeOut();
        } else {
            bubble.fadeIn().delay(3000 + msg.length * 200).fadeOut();
        }
        bubble.text(msg);
        if (payload.speak === true) {
            
            if (language === 'en-US') {
                responsiveVoice.speak(msg, "UK English Male",  {pitch:9},{volume: 1},{rate: 10});
            } 
            else if (language === 'fr-FR') {
                responsiveVoice.speak(msg, "French Female",  {pitch:1.6},{volume: 1},{rate: 10});
            }
            else if (language === 'it-IT') {
                responsiveVoice.speak(msg, "Italian Male",  {pitch:1.6},{volume: 1},{rate: 10});
            }
        }
    }
    else if (message.type === RESET) {
        $('#loading_modal').modal('hide');
        $('#search_modal').modal('hide');
        $('#result_modal').modal('hide');
        $('#mic_icon').removeClass('blinking');
    }

    
    else if (message.type === DISPLAY_GO) {
        $('#go_modal').modal('show');
        
        if (payload.display === true) {
            $('#go_modal').modal('show');
        }
        else {
            $('#go_modal').modal('hide');
        }
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
                'title': textfield.val()
            }
        }) 
    });  
    UI.publish(UI_msg);
    textfield.val("");
}

function goButtonCallBack(){
    var UI_msg = new ROSLIB.Message({
        type: LETS_GO
    });
    UI.publish(UI_msg);
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
        cell.setAttribute('class', 'align-middle thumbnail p-1');
        cell.innerHTML = '<img class="img-rounded img-responsive" src="' + book['thumbnail'] + '" />';

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

function foundMessage(language, title, author) {
    var txt = 'Here is what I found';
    if (language == 'en-US'){
        if (title != ''){
            txt += ' for ' + title;
        }
        if (author != ''){
            txt += ' by ' + author;
        }
    }
    else if (language == 'fr-FR'){
        var txt = "Voici ce que j'ai trouvé";
        if (title != ''){
            txt += ' pour ' + title;
        }
        if (author != ''){
            txt += ' de ' + author;
        }
    }
    else if (language == 'it-IT'){
        var txt = "Questo è ciò che ho trovato";
        if (title != ''){
            txt += ' per ' + title;
        }
        if (author != ''){
            txt += ' di ' + author;
        }
    }
    return txt;
}

function notFoundMessage(language, title, author){
    var txt = 'Sorry, I did not find any book';
    if (language == 'en-US'){
        if (title != ''){
            txt += ' for ' + title;
        }
        if (author != ''){
            txt += ' by ' + author;
        }
    }
    else if (language == 'fr-FR'){
        var txt = "Pardon, je n'ai pas trouvé de livre";
        if (title != ''){
            txt += ' pour ' + title;
        }
        if (author != ''){
            txt += ' de ' + author;
        }
    }
    else if (language == 'it-IT'){
        var txt = "Non ho trovato nulla";
        if (title != ''){
            txt += ' per ' + title;
        }
        if (author != ''){
            txt += ' di ' + author;
        }
    }
    return txt;
}

