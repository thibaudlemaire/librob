# Types :
int8 SEARCH_RESPONSE = 1 # Payload must be a JSON formated list, see below
int8 LOADING = 2
int8 COMMUNICATION = 3
int8 DIRECTION = 6 # Payload must be a dict with a "direction" (JSON string) key
int8 LISTENING = 7
int8 RESET = 8


# SEARCH_RESPONSE example payload :
# { books: [
#   {   title: "Machine learning",
#        author: "Mitchell, Tom M. (Tom Michael)",
#        code: "006.31 MIT",
#        floor: 1,
#        available: true },
#    {   title: "Red seas under red skies",
#        author: "Lynch, Scott",
#        code: "800 LYN"
#        floor: 5,
#        available: true }
#   ]
# }

# Fields
Header header
int8 type       # Need a Type constant
string payload  # Data, JSON formated
