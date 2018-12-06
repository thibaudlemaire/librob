from __future__ import unicode_literals
import io
import json
from snips_nlu import SnipsNLUEngine, load_resources
from snips_nlu.default_configs import CONFIG_EN

class NLP():

    def __init__(self):
        # load language resources
        load_resources(u"en")

        # create NLU Engine
        self.engine = SnipsNLUEngine(config=CONFIG_EN)
        # train engine
        """
        with io.open("snips/nlp_db.json") as f:
            dataset = json.load(f)
        print('start training')
        self.engine.fit(dataset=dataset)
        print('finished training')
        self.engine.persist('nlu_engine')
        """  
        engine_location = rospy.get_param("~engine_location")

        self.engine = SnipsNLUEngine.from_path(engine_location)
        print('snips engine ready')
        
        
    def parse(self, txt):
        
        parsed = self.engine.parse(txt.decode("utf-8"))
        #print(json.dumps(parsed['slots'], indent=2))
        
        slots = parsed['slots']
        pair = dict()
  
        try:
            for slot in slots:
                pair[slot['slotName'].encode("ascii")] = slot['value']['value'].encode("ascii")
            return True, pair          
        except Exception as e:
            return False, e
        
        


if __name__ == '__main__':
    nlp = NLP()    
    stop = False
    while not stop:
        request = raw_input('input request:\n')
        if request == 'stop':
            stop = True
        else:                  
            success, data = nlp.parse(request)
            if success:
                print(data)
            else:
                print('error:', data)
            
        