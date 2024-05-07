import oracle

# property to verify
PROPERTY =  "({topic: 'agLand', data_mod: 'safetyLanding'} -> once[2:5]{topic: 'battery', battery: 'Safety'}) and (not {topic: 'agLand', data_mod: 'safetyLanding'} -> (once[2:5]{topic: 'agLand', data_mod: 'safetyLanding'} or not (once[3:5] {topic: 'battery', battery: 'Safety'})))"

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
    if message['topic'] == "agLand":
        predicates['data_mod'] = str(message['data'].replace('"',''))

    if message['topic'] == "battery":
    	percentage = int(message['data'])
    	if percentage >= 20 and percentage <= 40: 
    		predicates['battery'] = 'Safety'
    	elif percentage < 20:
    		predicates['battery'] = 'Critical'
    	else:
    		predicates['battery'] = 'Unspecified'
    		   
    return predicates
