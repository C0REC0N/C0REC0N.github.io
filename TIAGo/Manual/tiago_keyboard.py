from pynput import keyboard
import subprocess

# Base Movement Commands

linear_speed = 0.0  # Forward/Backward movement
angular_speed = 0.0  # Left/Right rotation
speed = 0.2 # Step size for speed changes

def move_base(count):
    """Send velocity command to Tiago's base."""
    command = ["timeout", f"{count+2}", "rostopic", "pub", "--rate", "10", "/mobile_base_controller/cmd_vel", "geometry_msgs/Twist", f"""
linear:
  x: {linear_speed}
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: {angular_speed}"""]
    
    process = subprocess.Popen(command)
    check_for_stop(process)
    reset_speed()

def reset_speed():
    global linear_speed
    global angular_speed
    linear_speed = 0.0
    angular_speed = 0.0

def moveForward(count):
    global linear_speed
    global speed
    linear_speed = speed
    print("Moving Forward")
    move_base(count)

def moveBackward(count):
    global linear_speed
    global speed
    linear_speed = -speed
    print("Moving Backward")
    move_base(count)

def turnLeft(count):
    global angular_speed
    global speed
    angular_speed = speed
    print("Turning Left")
    move_base(count)

def turnRight(count):
    global angular_speed
    global speed
    angular_speed = -speed
    print("Turning Right")
    move_base(count)
    
# Head Movement Commands

pan = 0.0  # Left (-) / Right (+)
tilt = 0.0  # Down (+) / Up (-)
head_step = 0.3  # Step size for each movement

def move_head():
    global pan
    global tilt
    """Send the head position command to Tiago."""
    if pan != 0:
        command = f"rosrun play_motion move_joint head_1_joint {pan} 2.0"
    else:
        command = f"rosrun play_motion move_joint head_2_joint {tilt} 2.0"
    
    subprocess.run(command, shell=True, text=True)
    resetHead()

def resetHead():
    global tilt
    global pan
    pan = 0.0
    tilt = 0.0

def headUp(count):
    global tilt
    global head_step
    tilt += head_step*count
    print("Looking Up")
    move_head()

def headDown(count):
    global tilt
    global head_step
    tilt -= head_step*count
    print("Looking Down")
    move_head()

def headLeft(count):
    global pan
    global head_step
    pan += head_step*count
    print("Looking Left")
    move_head()

def headRight(count):
    global pan
    global head_step
    pan -= head_step*count
    print("Looking Right")
    move_head()
    
    
# Moving Torso Commands

torso_pos = 0.15  # 0.0 (down) to 0.34 (up)
step = 0.05  # How much to move per key press

def move_torso():
    global torso_pos
    """Send the torso lift position command to Tiago."""
    command = f"rosrun play_motion move_joint torso_lift_joint {torso_pos} 2.0"
    subprocess.run(command, shell=True, text=True)

# Define key actions
def torsoUp():
    global torso_pos
    if torso_pos < 0.35:
        torso_pos += step
        print("Rising")
        move_torso()

def torsoDown():
    global torso_pos
    if torso_pos > 0.0:
        torso_pos -= step
        print("Lowering")
        move_torso()

def resetTorso():
    global torso_pos
    torso_pos = 0.0
    print("Resetting Torso")
    move_torso()
        
# Basic Arm Command  

def tighten():
    command = "rosrun play_motion run_motion_python_node.py close_right"
    print("Tightening Grip") 
    speak("Caution: Tightening Right Grip")
    subprocess.run(command, shell=True, text=True)

def release():
    command = "rosrun play_motion run_motion_python_node.py open_right"
    print("Releasing Grip") 
    speak("Caution: Releasing Right Grip")  
    subprocess.run(command, shell=True, text=True)
    
def offer():
    command = "rosrun play_motion run_motion_python_node.py offer_right"
    print("Offering Hand")
    speak("Caution: Offering Right Hand")
    subprocess.run(command, shell=True, text=True)
    offerPos()

def home_right():
    command = "rosrun play_motion run_motion_python_node.py home_right"
    print("Retracting Hand")
    speak("Caution: Retracting Right Hand")
    subprocess.run(command, shell=True, text=True)


# Advanced Arm Control
arm_pos = [-1.1,1.46,2.71,1.7,-1.57,1.38,-0.0002,0.0,0.0]
arm_step = 0.3
gripper_step = 0.025
joint = 0

def resetPos():
    global arm_pos
    arm_pos = [-1.1,1.46,2.71,1.7,-1.57,1.38,-0.0002,0.0,0.0]

def offerPos():
    global arm_pos
    arm_pos = [1.5,0.57,0.06,1.0006,-1.7,-1.0,-4.91,0.0,0.0]

def arm_command():
    command = f"rosrun play_motion move_joint arm_right_{joint}_joint {arm_pos[joint-1]} 5.0"
    subprocess.run(command, shell=True, text=True)

def grip_command():
    if joint == 8:
        command = f"rosrun play_motion move_joint gripper_right_left_finger_joint {arm_pos[joint-1]} 5.0"
    if joint == 9:
        command = f"rosrun play_motion move_joint gripper_right_right_finger_joint {arm_pos[joint-1]} 5.0"
    subprocess.run(command, shell=True, text=True)

def joint_select():
    global joint
    running = True
    while (running):
        user_input = input("Select Joint via Number. You cannot control the rest of the robot. Press q to return\n")
        if len(user_input) == 1:
            if user_input == 'q':
                running = False
            else:
                joint = int(user_input)
                if (joint >0) & (joint <=7):
                    arm_control()
                if (joint >7) & (joint <=9):
                    grip_control()
        

def arm_control():
    global arm_pos
    global arm_step
    running = True
    print(f"Controlling Joint {joint}")
    while(running):
        user_input = input("Use r and t to control. You cannot control the rest of the robot. Press q to return\n")
        if check(user_input):
            if user_input == 'q':
                running = False
            if user_input[0] == 't':
                arm_pos[joint-1] -= arm_step*len(user_input)    
                arm_command()
            if user_input[0] == 'r':
                arm_pos[joint-1] += arm_step*len(user_input)
                arm_command()

def grip_control():
    global arm_pos
    global arm_step
    running = True
    print(f"Controlling Grip {joint}")
    while(running):
        user_input = input("Use r and t to control. You cannot control the rest of the robot. Press q to return\n")
        if len(user_input) == 1:
            if check(user_input):
                if user_input == 'q':
                    running = False
                if user_input == 't':
                    arm_pos[joint-1] -= gripper_step
                    grip_command()
                if user_input == 'r':
                    arm_pos[joint-1] += gripper_step
                    grip_command()



      
# Speech Command       
def speak(text):
    command = f"""timeout 3 rostopic pub -1 /tts/goal pal_interaction_msgs/TtsActionGoal "{{
                    goal: {{
                        rawtext: {{
                            text: '{text}',
                            lang_id: 'en_GB'
                        }}
                    }}
                }}" """
    print("Tiago is Speaking")
    subprocess.run(command, shell=True, text=True)

# Home
def home():
    command = "rosrun play_motion  run_motion_python_node.py home"
    print("Returning to home position")
    speak("Caution: Return to Home Position")
    subprocess.run(command, shell=True, text=True)

    
# Sending Commands
def check(text):
    first = text[0]
    for letter in text:
        if letter != first:
            return False
    
    return True

def check_for_stop(process):
    listen = input("Send E to halt command, Send EE to full emergency stop\n")
    if listen == "E":
        process.terminate()


def runKeyboard():
    running = True
    while (running):
        user_input = input("Enter command. Send 'E' to exit program\n")
        if len(user_input) != 0:
            if check(user_input):
                char = user_input[0]
                count = len(user_input)
                if user_input == 'q':
                    joint_select()
                if char == "w":
                    moveForward(count)
                if char == 'a':
                    turnLeft(count)
                if char == 's':
                    moveBackward(count)
                if char == 'd':
                    turnRight(count)
                if user_input == 'y':
                    speak("Hello")  
                if char == 'i':
                    headUp(count)
                if char == 'j':
                    headLeft(count)
                if char == 'k':
                    headDown(count)
                if char == 'l':
                    headRight(count)
                if user_input == 'R':
                    release()
                if user_input == 'T':
                    tighten()
                if user_input == 'W':
                    torsoUp()
                if user_input == 'S':
                    torsoDown()
                if user_input == 'F':
                    resetTorso()
                if user_input == 'F':
                    offer()
                if user_input == 'G':
                    home_right()
                if user_input == 'H':
                    home()
                    resetPos()
                if char == "E":
                    running = False
                    print("Exited Program")
            else:
                print("Please only send one command at a time")
        else:
            print("Please enter a command")

if __name__ == "__main__":
    runKeyboard()
