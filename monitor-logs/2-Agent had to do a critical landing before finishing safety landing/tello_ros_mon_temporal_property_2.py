import oracle

# property to verify
PROPERTY =  "(once[20:21]{topic: 'battery', battery: 'Safety'} -> {topic: 'agLand', data: 'safetyLanding'})"

# predicates used in the property (initialization for time 0)
predicates = dict()

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    print(message)
    predicates = dict()

    predicates['topic'] = message['topic']
    predicates['time'] = str(message['time'])
    
    if message['topic'] in ["agLand", "battery"]:
        predicates['data'] = str(message['data'])
    
    if message['topic'] == "battery":
    	percentage = int(message['data'])
    	if percentage >= 20 and percentage <= 40: 
    		predicates['battery'] = 'Safety'
    	elif percentage < 20:
    		predicates['battery'] = 'Critical'
    	else:
    		predicates['battery'] = 'Unspecified'
    		
   
    return predicates
