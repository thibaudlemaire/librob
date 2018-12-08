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



var COMMUNICATION_MESSAGE = new Object();
var ENGLISH_MESSAGES = new Object();
var FRENCH_MESSAGES = new Object();
ENGLISH_MESSAGES.SEARCHING = "Hmm... let me think";
ENGLISH_MESSAGES.DB_ERROR = "Sorry, I cannot request the database";
ENGLISH_MESSAGES.LOCATOR_ERROR = "Sorry, I cannot find the location of this book";
ENGLISH_MESSAGES.NOT_UNDERSTOOD = "Sorry, I did not understand what you said";
ENGLISH_MESSAGES.FOLLOW_ME = "Let's go ! Please follow me";
ENGLISH_MESSAGES.BUSY = "Sorry, I'm still moving and cannot process your request";
ENGLISH_MESSAGES.HOW_TO_TALK = "I'm listening, what can I do for you ?";
ENGLISH_MESSAGES.READY = "I'm ready !";
ENGLISH_MESSAGES.TIME_OUT = "You've been too long...";
FRENCH_MESSAGES.SEARCHING = "Hmm... Laissez-moi voir";
FRENCH_MESSAGES.DB_ERROR = "Pardon, je n'arrive pas à accéder la base de données";
FRENCH_MESSAGES.LOCATOR_ERROR = "Pardon, je ne trouve pas la location de votre livre";
FRENCH_MESSAGES.NOT_UNDERSTOOD = "Pardon, je ne vous ai pas compris";
FRENCH_MESSAGES.FOLLOW_ME = "Allons-y! Veuillez me suivre s'il vous plait";
FRENCH_MESSAGES.BUSY = "Pardon, je suis en mouvement et ne peux pas traiter votre demande";
FRENCH_MESSAGES.HOW_TO_TALK = "Je vous écoute, que puis-je faire pour vous ?";
FRENCH_MESSAGES.READY = "Je suis prêt !";
FRENCH_MESSAGES.TIME_OUT = "Vous avez pris trop de temps..";
COMMUNICATION_MESSAGE.en = ENGLISH_MESSAGES;
COMMUNICATION_MESSAGE.fr = FRENCH_MESSAGES;



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

        var msg = ''
        selector = document.getElementById('selector');
        language = selector.options[selector.selectedIndex].value;

        if (type == 'FOUND'){
            msg = foundMessage(language, title, author);
        }
        else if (type == 'NOT_FOUND'){
            msg = notFoundMessage(language, title, author);
        }
        else {
            if (language == 'en-US') {
                msg = COMMUNICATION_MESSAGE['en'][type];
                console.log(msg)
            }
            else if (language == 'fr-FR') {
                msg = COMMUNICATION_MESSAGE['fr'][type];
            }
        }
     
        var bubble = $('#speech-bubble');
        if (bubble.is(':visible')) {
            bubble.stop(true).show().delay(3000 + msg.length * 50).fadeOut();
        } else {
            bubble.fadeIn().delay(3000 + msg.length * 50).fadeOut();
        }
        bubble.text(msg);
        if(payload.speak === true) {
            if (language == 'en-US') {
                responsiveVoice.speak(msg, "UK English Male",  {pitch:9},{volume: 1},{rate: 10});
            } 
            else if (language == 'fr-FR') {
                responsiveVoice.speak(msg, "French Female",  {pitch:1.6},{volume: 1},{rate: 10});
            }
        }
    }
    else if (message.type === RESET) {
        $('#loading_modal').modal('hide');
        $('#search_modal').modal('hide');
        $('#result_modal').modal('hide');
        $('#mic_icon').removeClass('blinking');
    }

    /*
    else if (message.type === DISPLAY_GO) {
        
    }*/

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
    return txt;
}
