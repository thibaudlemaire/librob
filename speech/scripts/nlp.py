from __future__ import unicode_literals
import io
import json
from snips_nlu import SnipsNLUEngine, load_resources
from snips_nlu.default_configs import CONFIG_EN

class NLP():

    def __init__(self):

        # load language specific resources
        load_resources(u"en")

        # create a Snips NLU Engine
        self.engine = SnipsNLUEngine(config=CONFIG_EN)

        # train the engine
        with io.open("librob.json") as f:
            dataset = json.load(f)
        self.engine.fit(dataset)
        print('NLP class ready!')

    def parse(self):
        
        #parse
        request = raw_input('what do you want?\n')
        parsed = self.engine.parse(request.decode("utf-8"))
        print(json.dumps(parsed, indent=2))


if __name__ == '__main__':
    nlp = NLP()
    nlp.parse()