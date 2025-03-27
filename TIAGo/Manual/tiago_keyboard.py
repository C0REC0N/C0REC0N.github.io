from pynput import keyboard
import subprocess

# ========== Global Parameters ========== #

# Base movement
linear_speed = 0.0
angular_speed = 0.0
speed = 0.2  # Step size for speed changes

# Head movement
pan = 0.0
tilt = 0.0
head_step = 0.3

# Torso movement
torso_pos = 0.15
step = 0.05  # Step size for torso adjustments

# Arm movement
arm_pos = [-1.1, 1.46, 2.71, 1.7, -1.57, 1.38, -0.0002, 0.0, 0.0]
arm_step = 0.3
gripper_step = 0.025
joint = 0

# ========== Base Movement Commands ========== #

def move_base(count):
    """Send base velocity command."""
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
    """Reset base speed to 0."""
    global linear_speed, angular_speed
    linear_speed = 0.0
    angular_speed = 0.0

def moveForward(count):
    global linear_speed
    linear_speed = speed
    print("Moving Forward")
    move_base(count)

def moveBackward(count):
    global linear_speed
    linear_speed = -speed
    print("Moving Backward")
    move_base(count)

def turnLeft(count):
    global angular_speed
    angular_speed = speed
    print("Turning Left")
    move_base(count)

def turnRight(count):
    global angular_speed
    angular_speed = -speed
    print("Turning Right")
    move_base(count)

# ========== Head Movement Commands ========== #

def move_head():
    """Send pan or tilt command to head."""
    global pan, tilt
    if pan != 0:
        command = f"rosrun play_motion move_joint head_1_joint {pan} 2.0"
    else:
        command = f"rosrun play_motion move_joint head_2_joint {tilt} 2.0"
    subprocess.run(command, shell=True, text=True)
    resetHead()

def resetHead():
    """Reset head orientation."""
    global tilt, pan
    pan = 0.0
    tilt = 0.0

def headUp(count):
    global tilt
    tilt += head_step * count
    print("Looking Up")
    move_head()

def headDown(count):
    global tilt
    tilt -= head_step * count
    print("Looking Down")
    move_head()

def headLeft(count):
    global pan
    pan += head_step * count
    print("Looking Left")
    move_head()

def headRight(count):
    global pan
    pan -= head_step * count
    print("Looking Right")
    move_head()

# ========== Torso Movement Commands ========== #

def move_torso():
    """Send torso lift command."""
    command = f"rosrun play_motion move_joint torso_lift_joint {torso_pos} 2.0"
    subprocess.run(command, shell=True, text=True)

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

# ========== Basic Arm Commands ========== #

def tighten():
    print("Tightening Grip") 
    speak("Caution: Tightening Right Grip")
    subprocess.run("rosrun play_motion run_motion_python_node.py close_right", shell=True, text=True)

def release():
    print("Releasing Grip") 
    speak("Caution: Releasing Right Grip")
    subprocess.run("rosrun play_motion run_motion_python_node.py open_right", shell=True, text=True)

def offer():
    print("Offering Hand")
    speak("Caution: Offering Right Hand")
    subprocess.run("rosrun play_motion run_motion_python_node.py offer_right", shell=True, text=True)
    offerPos()

def home_right():
    print("Retracting Hand")
    speak("Caution: Retracting Right Hand")
    subprocess.run("rosrun play_motion run_motion_python_node.py home_right", shell=True, text=True)

# ========== Advanced Arm Control ========== #

def resetPos():
    global arm_pos
    arm_pos = [-1.1, 1.46, 2.71, 1.7, -1.57, 1.38, -0.0002, 0.0, 0.0]

def offerPos():
    global arm_pos
    arm_pos = [1.5, 0.57, 0.06, 1.0006, -1.7, -1.0, -4.91, 0.0, 0.0]

def arm_command():
    """Send command to control selected joint."""
    command = f"rosrun play_motion move_joint arm_right_{joint}_joint {arm_pos[joint-1]} 5.0"
    subprocess.run(command, shell=True, text=True)

def grip_command():
    """Send command to control selected gripper."""
    if joint == 8:
        command = f"rosrun play_motion move_joint gripper_right_left_finger_joint {arm_pos[joint-1]} 5.0"
    if joint == 9:
        command = f"rosrun play_motion move_joint gripper_right_right_finger_joint {arm_pos[joint-1]} 5.0"
    subprocess.run(command, shell=True, text=True)

def joint_select():
    """Prompt user to select a joint to control."""
    global joint
    running = True
    while running:
        user_input = input("Select Joint via Number. You cannot control the rest of the robot. Press q to return\n")
        if len(user_input) == 1:
            if user_input == 'q':
                running = False
            else:
                joint = int(user_input)
                if 0 < joint <= 7:
                    arm_control()
                if 7 < joint <= 9:
                    grip_control()

def arm_control():
    """Control arm joints via keyboard input."""
    global arm_pos
    print(f"Controlling Joint {joint}")
    running = True
    while running:
        user_input = input("Use r and t to control. You cannot control the rest of the robot. Press q to return\n")
        if check(user_input):
            if user_input == 'q':
                running = False
            elif user_input[0] == 't':
                arm_pos[joint-1] -= arm_step * len(user_input)
                arm_command()
            elif user_input[0] == 'r':
                arm_pos[joint-1] += arm_step * len(user_input)
                arm_command()

def grip_control():
    """Control gripper joints via keyboard input."""
    global arm_pos
    print(f"Controlling Grip {joint}")
    running = True
    while running:
        user_input = input("Use r and t to control. You cannot control the rest of the robot. Press q to return\n")
        if len(user_input) == 1 and check(user_input):
            if user_input == 'q':
                running = False
            elif user_input == 't':
                arm_pos[joint-1] -= gripper_step
                grip_command()
            elif user_input == 'r':
                arm_pos[joint-1] += gripper_step
                grip_command()

# ========== Speech Commands ========== #

def speak(text):
    """Send text-to-speech command."""
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

# ========== Home Command ========== #

def home():
    """Return robot to home position."""
    print("Returning to home position")
    speak("Caution: Return to Home Position")
    subprocess.run("rosrun play_motion  run_motion_python_node.py home", shell=True, text=True)

# ========== Command Helpers ========== #

def check(text):
    """Check if input contains the same repeated character."""
    return all(letter == text[0] for letter in text)

def check_for_stop(process):
    """Allow user to halt ongoing base movement."""
    listen = input("Send E to halt command, Send EE to full emergency stop\n")
    if listen == "E":
        process.terminate()

# ========== Keyboard Input Handler ========== #

def runKeyboard():
    """Main keyboard input loop."""
    running = True
    while running:
        user_input = input("Enter command. Send 'E' to exit program\n")
        if user_input:
            if check(user_input):
                char = user_input[0]
                count = len(user_input)

                if user_input == 'q': joint_select()
                elif char == 'w': moveForward(count)
                elif char == 'a': turnLeft(count)
                elif char == 's': moveBackward(count)
                elif char == 'd': turnRight(count)
                elif user_input == 'y': speak("Hello")
                elif char == 'i': headUp(count)
                elif char == 'j': headLeft(count)
                elif char == 'k': headDown(count)
                elif char == 'l': headRight(count)
                elif user_input == 'R': release()
                elif user_input == 'T': tighten()
                elif user_input == 'W': torsoUp()
                elif user_input == 'S': torsoDown()
                elif user_input == 'F': 
                    offer()
                    offerPos()
                elif user_input == 'G': 
                    home_right()
                    resetPos()
                elif user_input == 'H':
                    home()
                    resetPos()
                elif char == 'E':
                    running = False
                    print("Exited Program")
            else:
                print("Please only send one command at a time")
        else:
            print("Please enter a command")

if __name__ == "__main__":
    runKeyboard()
