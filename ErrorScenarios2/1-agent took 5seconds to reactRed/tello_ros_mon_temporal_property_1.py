import oracle

# property to verify
PROPERTY = "({topic: 'agentReact', data: 'reactRed'} -> once[2:3]{topic: 'detectRed', detectRed: 'True'})"

# predicates used in the property (initialization for time 0)
predicates = dict()

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    print(message)
    predicates = dict()

    predicates['topic'] = message['topic']
    predicates['time'] = str(message['time'])

    if message['topic'] in ["agentReact", "detectRed"]:
        predicates['data'] = str(message['data'])
        
    if message['topic'] == "detectRed":
    	detectRed = int(message['data'])
    	predicates['detectRed'] = str((detectRed > 200))
    	
    return predicates
