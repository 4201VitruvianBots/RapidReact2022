import json

# ==== CONFIG ====
pathname = "FourBallAuto"
splits = [0,1,2,4]
format = False
# ================

# File reading
input_file = open(pathname + "-Master.path", "rt")
input_text = input_file.read()
master_path = json.loads(input_text)['waypoints']

splits.append(len(master_path)-1)

# Extract paths
for i in range(len(splits)-1):
    # Extract points
    waypoints = [master_path[j].copy() for j in range(splits[i], splits[i+1]+1)]
    for j in waypoints:
        print('(', j['anchorPoint']['x'], ', ', j['anchorPoint']['y'], ')', sep='')

    # Format for single path
    waypoints[0]['prevControl'] = None
    waypoints[0]['isReversal'] = False
    waypoints[len(waypoints)-1]['nextControl'] = None
    waypoints[len(waypoints)-1]['isReversal'] = False

    # Write to file
    output_path = {'waypoints':waypoints}
    output_file = open(pathname + "-" + str(i+1) + ".path", "w")
    output_file.write(json.dumps(output_path, indent=4)) if format else output_file.write(json.dumps(output_path))
    output_file.close()

    print()

print('Make sure to check your inversions!', end='')
input()
