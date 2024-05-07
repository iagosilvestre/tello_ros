import oracle

# property to verify
PROPERTY = "(forall[t]. {topic: 'cmd_vel', time: *t} -> {topic: 'cmd_vel', forwardMotion: 'True', time: *t})"

# predicates used in the property (initialization for time 0)
predicates = dict()

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    print(message)
    predicates = dict()

    predicates['topic'] = message['topic']
    predicates['time'] = str(message['time'])
    
    if message['topic'] == 'cmd_vel':
    	linearY = float(message.linear.y)
    	predicates['forwardMotion'] = str((linearY >= 0))
    	
    #print(predicates)
    return predicates
