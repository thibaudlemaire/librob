#!/usr/bin/env python
import rospy
import json
from librarian_msgs.msg import UI, UI_feedback  # from package import message
import speech_recognition as sr


# UI is the actual message
class Speech:
    def __init__(self):
        rospy.init_node('speech_node', anonymous=True)
        self.pub_command = rospy.Publisher('ui_command', UI,
                                           queue_size=10)  # speechrequest is the name of the topic you publishing on,UI is our custom message
        self.pub_feedback = rospy.Publisher('ui_feedback', UI_feedback, queue_size=10)
        self.r = sr.Recognizer()
        self.mic = sr.Microphone(2, 44100)
        self.r.energy_threshold = 500

        print(self.mic.SAMPLE_RATE)
        print(self.mic.format)

        self.r.dynamic_energy_threshold = True

        print('Speech node ready')

    def recogTest(self, source, language):
        print('Speech starts listening')
        try:
            audio = self.r.listen(source, timeout=2.0, phrase_time_limit=3.0)
            # audio = self.r.record(source)
            #f = open('/home/ubuntu/foo.wav', 'wb')
            #f.write(audio.get_wav_data())
            #f.close()
            print('Speech starts recognition')
            text = self.r.recognize_google(audio, language=language)
            print('Speech stops recognition')
            return True, text
        except Exception as e:
            return False, e

    def publish(self, topic, msg_type, msg_payload):
        if topic == 'ui_command':
            msg = UI()
            msg.type = msg_type
            msg.payload = msg_payload
            self.pub_command.publish(msg)
        elif topic == 'ui_feedback':
            msg = UI_feedback()
            msg.type = msg_type
            msg.payload = msg_payload
            self.pub_feedback.publish(msg)
        else:
            print('Speech error: attempt to publish on unknown topic.')

    # callback when node reecives a message on the ui_command topic
    def sub_command_callback(self, UI_msg):  # UI_msg: message given by the data that is passed

        if UI_msg.type == UI.SPEECH_TRIGGER:
            # start speech recognition

            language = str(json.loads(UI_msg.payload)['language'])

            with self.mic as source:
                # with sr.AudioFile("/home/ubuntu/foo.wav") as source:

                # Tell the UI_feedback that his speech is being listened with ui_feedback message of type LISTENING
                self.publish('ui_command', UI_feedback.LISTENING, '')

                recognised, txt = self.recogTest(source, language)

                if recognised:
                    self.publish('ui_command', UI.SEARCH_REQUEST, json.dumps({'request': txt}))
                    print('You said: ', txt)
                else:
                    self.publish('ui_feedback', UI.NOT_UNDERSTOOD, '')
                    print('Speech error:', txt)

    def startNode(self):
        # subscriber object of the ui_command topic
        self.sub_command = rospy.Subscriber('ui_command', UI, self.sub_command_callback)  # (topic, messsage, callback)
        rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    speech = Speech()
    speech.startNode()
