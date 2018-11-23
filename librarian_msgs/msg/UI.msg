# Types :
int8 SPEECH_TRIGGER = 1
int8 SEARCH_REQUEST = 10 # Payload must be a dict with a "request" (JSON string) key
int8 BOOK_CHOSEN = 20 # Payload must be a dict with a "chosen_code" (JSON string) key
int8 NOT_UNDERSTOOD = 30
int8 ERROR_NOT_FOUND = 404
int8 SUCCESS_FOUND = 200

# Fields
Header header
int8 type       # Need a Type constant
string payload  # Data, JSON formated
