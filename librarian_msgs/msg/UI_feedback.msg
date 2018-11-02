# Types :
int8 SEARCH_RESPONSE = 1 # Payload must be a dict with "books" as a list of dict with "title", "author", "code", "available" (bool) and floor (int) keys

# Fields
Header header
int8 type       # Need a Type constant
string payload  # Data, JSON formated
