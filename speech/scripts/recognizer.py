import speech_recognition as sr

class Recognizer():

    def __init__(self):
        self.r = sr.Recognizer()
        self.mic = sr.Microphone(device_index = 2, sample_rate = 44100, chunk_size = 512)
        self.r.dynamic_energy_threshold = True
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)

    def recognize(self, txt):
        print('Speech starts listening')
        try:
            audio = self.r.listen(source, timeout=2.0, phrase_time_limit=8.0)
            print('Speech starts recognition')
            text = self.r.recognize_google(audio, language=language)
            print('Speech stops recognition')
            return True, text
        except Exception as e:
            return False, e
    
        
if __name__ == '__main__':
    recognizer = Recognizer()

    with recognizer.mic as source:
        recognised, recog_txt = recognizer.recognize(source, 'en-US')
