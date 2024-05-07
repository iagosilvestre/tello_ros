import oracle

# property to verify
PROPERTY = '''({topic: 'agentReact', data_mod: 'reactRed'} -> once[1:3]{topic: 'detectRed', detectRed: 'True'}) and (not {topic: 'agentReact', data_mod: 'reactRed'} -> (once[1:3]{topic: 'agentReact', data_mod: 'reactRed'} or not (once[2:3]{topic: 'detectRed', detectRed: 'True'})))'''

# predicates used in the property (initialization for time 0)
predicates = dict()

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    print(message)
    predicates = dict()

    predicates['topic'] = message['topic']
    predicates['time'] = str(message['time'])
    #print(message['time'])
    #print(predicates['time'])

    if message['topic'] in ["agentReact", "detectRed"]:
        predicates['data'] = str(message['data'])
    if message['topic'] == "agentReact":
        predicates['data_mod'] = str(message['data'].replace('"', ''))

        
    if message['topic'] == "detectRed":
    	detectRed = int(message['data'])
    	predicates['detectRed'] = str((detectRed > 200))
    	#print('detect:', message)
    	#print(predicates['detectRed'])
    	
    return predicates
