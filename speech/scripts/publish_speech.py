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
        self.r = sr.Recognizer()
        print('node ready')

    def recogTest(self, source):
        print('start listening')
        try:
            audio = self.r.listen(source)
            print('start recognition')
            text = r.recognize_google(audio)
            print('stop recognition')
            return True, text
        except Exception as e:
            return False, e
        

    # callback when node reecives a message on the ui_command topic
    def sub_command_callback(self, UI_msg): # UI_msg: message given by the data that is passed

        if UI_msg.type == UI.SPEECH_TRIGGER:
            # start speech recognition

            with sr.Microphone() as source:
            
                # Tell the UI_feedback that his speech is being listened with ui_feedback message of type LISTENING
                msg = UI_feedback()
                msg.type = UI_feedback.LISTENING
                self.pub_feedback.publish(msg)

                self.r.adjust_for_ambient_noise(source)
                self.r.dynamic_energy_threshold = True

                recognised, txt = recogTest(source)

                if recognised:
                    msg = UI()
                    msg.type = UI.SEARCH_REQUEST
                    msg.payload = json.dumps({'request': txt})
                    self.pub_command.publish(msg)
                    print('you said:', txt)
                else:
                    msg = UI()
                    msg.type = UI.NOT_UNDERSTOOD
                    self.pub_feedback.publish(msg)
                    print('error:', txt)


        
            
    def startNode(self):
        # subscriber object of the ui_command topic
        self.sub_command = rospy.Subscriber('ui_command', UI, self.sub_command_callback) # (topic, messsage, callback)
        rospy.spin() # keeps python from exiting until this node is stopped




if __name__ == '__main__':
    speech = Speech()
    speech.startNode()