# lpython script for placing the aruco markers in a random place within the field

# import random to generate random number
import random 
import sys #sys.argv for arguments

# we will append <pose></pose> tags to the aruco marker visual tag in the .world file
if len(sys.argv) > 1:
    world_path = f"/home/enreysen/projectRoot/project_ws/src/gazebo_drone/worlds/{sys.argv[1]}.world"
    file =  open(world_path, 'r')
    lines = file.readlines()

    index = 0
    for line in lines:
        # I placed a marker before each <pose> tag of the aruco marker to signify that it can be changed
        if "<!--aruco pose -->" in line:

            # generate a random float for x and y coordinates between
            # -/+ ((size of the field) / 2) - ((size of aruco marker) / 2) --> only spawn within the field
            # (9.144 / 2) - (0.3048 / 2) --> between -4.4196 and 4.4196
            # round to 2 decimal places

            x_coord = round(random.uniform(-4.4196, 4.4196), 2)
            y_coord = round(random.uniform(-4.4196, 4.4196), 2)

            newline = f"          <!--aruco pose --><pose>{x_coord} {y_coord} .005 0 0 0</pose>\n"
            
            # replace line with new coordinates
            lines[index] = newline
        
            print(f"Aruco marker placed at ({x_coord}, {y_coord}, 0.005)")

        index += 1

    file.close()

    # Actually writes the changes to the world file
    final_file = open(world_path, 'w')
    final_file.writelines(lines)
else:
    print("You must include only the filename as a command-line argument. \nFor example: python random_placement.py challenge_1")


