# States
int8 IDLE_STATE = 0
int8 INITIAL_INTERACTION_STATE = 1      # User interacting with the robot (choosing a book for example)
int8 MOVING_TO_LIFT = 2
int8 WAITING_LIFT = 3
int8 JUMPING_IN_LIFT = 4
int8 WAITING_IN_LIFT = 5
int8 MOOVING_TO_GOAL = 6
int8 BOOK_REACHED = 7

# Fields
Header header
int8 state
