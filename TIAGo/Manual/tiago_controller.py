import pygame
pygame.quit()
import subprocess
import time

joystick = None

def setup():
    global joystick
    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()

    # Check if a joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Please connect a Logitech controller.")
        exit()

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")

linear_speed = 0.0
angular_speed = 0.0

def move_base():
    global linear_speed
    global angular_speed
    """Send velocity command to Tiago's base."""
    command = ["timeout", "3", "rostopic", "pub", "--rate", "10", "/mobile_base_controller/cmd_vel", "geometry_msgs/Twist", f"""
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
    
def check_for_stop(process):
    
    running = True
    while (running):
        for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:  # Check if a button is pressed
                    button = event.button
                    if button == 2:
                        running = False
                        process.terminate()
                if event is not None:
                    running = False

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

def home_right():
    command = "rosrun play_motion run_motion_python_node.py home_right"
    print("Retracting Hand")
    speak("Caution: Retracting Right Hand")
    subprocess.run(command, shell=True, text=True)

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

pan = 0
tilt = 0

def move_head():
    global pan
    global tilt
    """Send the head position command to Tiago."""
    if pan != 0:
        command = f"rosrun play_motion move_joint head_1_joint {pan} 1.0"
    else:
        command = f"rosrun play_motion move_joint head_2_joint {tilt} 1.0"
    
    subprocess.run(command, shell=True, text=True)
    pan = 0.0
    tilt = 0.0
