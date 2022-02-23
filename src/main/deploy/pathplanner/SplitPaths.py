import json

filename = "FourBallAuto"
splits = [0,1,2,4]

input_file = open(filename + "-Master.path", "rt")
input_text = input_file.read()
master_path = json.loads(input_text)['waypoints']

splits.append(len(master_path)-1)

for i in range(len(splits)-1):
    waypoints = [master_path[j] for j in range(splits[i], splits[i+1]+1)]
    
    waypoints[0]['prevControl'] = None
    waypoints[0]['isReversal'] = False
    waypoints[len(waypoints)-1]['nextControl'] = None            
    waypoints[len(waypoints)-1]['isReversal'] = False            

    output_path = {'waypoints':waypoints}

    output_file = open(filename + "-" + str(i+1) + ".path", "w")
    
    output_file.write(json.dumps(output_path))
    output_file.close()
    