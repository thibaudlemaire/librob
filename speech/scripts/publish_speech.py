#!/usr/bin/env python
import rospy
import json
from librarian_msgs.msg import UI, UI_feedback #from package import message
import speech_recognition as sr

#UI is the actual message
class Speech:
    def __init__(self):
        rospy.init_node('speech_node', anonymous=True)
        self.pub_command = rospy.Publisher('ui_command', UI, queue_size=10) #speechrequest is the name of the topic you publishing on,UI is our custom message
        self.pub_feedback = rospy.Publisher('ui_feedback', UI_feedback, queue_size=10)

    def callback(self, UI_msg): #message given by the data that is passed in this case UI_msg
        #UI_msg is the variable giving the function to the message
        if UI_msg.type == UI.SPEECH_TRIGGER:

            r=sr.Recognizer()

            with sr.Microphone() as source:
                print('start speaking')
                
                # Tell the UI_feedback that his speech is being listened
                msg = UI_feedback()
                msg.type = UI_feedback.LISTENING
                self.pub_feedback.publish(msg)

                try:
                    audio=r.listen(source)
                    text=r.recognize_google(audio)
                    # Send a search request to the UI
                    msg = UI()
                    msg.type = UI.SEARCH_REQUEST
                    msg.payload = json.dumps({'request': text})
                    self.pub_command.publish(msg)

                    print('you said:', text)

                except:
                    msg = UI_feedback()
                    msg.type = UI_feedback.NOT_UNDERSTOOD
                    self.pub_feedback.publish(msg)

                    print('Error: speech not recognised')
                    

            
    def receiver(self):
        self.sub_command = rospy.Subscriber('ui_command', UI, self.callback) #ui command is the topic and ui is the message
        rospy.spin() #keeps python from exiting until this node is stopped




if __name__ == '__main__':
    speech = Speech()
    speech.receiver()

#topic need to publish on and receive is /ui_command
#message to use ui
#speech reqquest and speech trigger are the types needed to change
