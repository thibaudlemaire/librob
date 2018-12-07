class Messages:
    SEARCHING = "Hmm... let me think"
    DB_ERROR = "Sorry, I cannot request the database"
    LOCATOR_ERROR = "Sorry, I cannot find the location of this book"
    NOT_UNDERSTOOD = "Sorry, I did not understand what you said"
    FOLLOW_ME = "Let's go ! Please follow me"
    BUSY = "Sorry, I'm still moving and cannot process your request"
    HOW_TO_TALK = "I'm listening, what can I do for you ?"
    READY = "I'm ready !"
    TIME_OUT = "You've been too long..."

    @staticmethod
    def NOT_FOUND(request_dict = {}):
        return "Sorry, I did not find any books" + Messages.build_text(request_dict)

    @staticmethod
    def FOUND(request_dict = {}):
        return "Here is what I found" + Messages.build_text(request_dict)

    @staticmethod
    def build_text(request_dict):
        search_text = ''
        if 'title' in request_dict:
            search_text += ' about ' + request_dict.get('title')
        if 'author' in request_dict:
            search_text += ' by ' + request_dict.get('author')
        return search_text
