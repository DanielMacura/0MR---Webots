import math
import time
from controller import Robot
from controller import Camera

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# References
base_motor = robot.getDevice('base_motor')
first_motor = robot.getDevice('first_motor')
second_motor = robot.getDevice('second_motor')
gripper = robot.getDevice('vacuum gripper')
camera = robot.getDevice('camera')

# Constants
zero = [0, 0, 0.03]    # Robot center
balls = [0.105, 0.105, 0.013]    # Ball slide

red_bowl = [0, 0.13, 0.3]
green_bowl = [0, -0.13, 0.3]
blue_bowl = [0.15, 0, 0.3]
a = 0.1  # Length of first arm
c = 0.14  # Length of second arm

# Set up devices
camera.enable(timestep)
camera.recognitionEnable(timestep)
camera.enableRecognitionSegmentation()

# Calculate angles for the arm
# Parameters: zero point of the arm base, plocation to transform to, length of first arm, lenght of second arm
# Return : theta, alpha, beta
#
# Source : https://www.alanzucconi.com/2020/09/14/inverse-kinematics-in-3d/
def inverse_kinematics_3d(zero, point, a, c):
    # Swap Y and Z coordinates
    zero = [zero[0], zero[2], zero[1]]
    point = [-point[0], -point[2], point[1]]
    
    # Calculate theta
    theta = math.atan2(point[2] - zero[2], point[0] - zero[0])

    # Calculate length of b
    b = math.sqrt((point[0] - zero[0])**2 + (point[2] - zero[2])**2)

    # Calculate A'
    delta_y = point[1] - zero[1]
    delta_w = math.sqrt((point[0] - zero[0])**2 + (point[1] - zero[1])**2 + (point[2] - zero[2])**2)
    A_prime = math.atan2(delta_y, delta_w)

    # Calculate A
    A = math.acos((b**2 + c**2 - a**2) / (2 * b * c)) + A_prime

    # Calculate B
    B = math.pi - math.acos((a**2 + c**2 - b**2) / (2 * a * c))

    return theta, A, B


i = 0
while robot.step(timestep) != -1:
    if i == 0:    # At zero - move to balls  
        theta, A, B = inverse_kinematics_3d(zero, balls, a, c)
        base_motor.setPosition(theta)
        first_motor.setPosition(A)
        second_motor.setPosition(B)
      
    if i == 100*4:    # A little bit after t0 - enable gripper
        gripper.turnOn()
    
    if i == 200*4 :    # Should be at balls, dtetect color, grab ball and move to bowl
        image = camera.getRecognitionSegmentationImageArray()
        if image:
            red   = image[0][0][0]
            green = image[0][0][1]
            blue  = image[0][0][2]
            if red == 255 and blue == 0 and green == 0:
                theta, A, B = inverse_kinematics_3d(zero, red_bowl, a, c)
                print("Detected red")
            elif red == 0 and blue == 255 and green == 0:
                theta, A, B = inverse_kinematics_3d(zero, blue_bowl, a, c)
                print("Detected blue")
            elif red == 0 and blue == 0 and green == 255:
                theta, A, B = inverse_kinematics_3d(zero, green_bowl, a, c)
                print("Detected green")
                
            # Move toward respective bowl
            base_motor.setPosition(theta)
            first_motor.setPosition(A)
            second_motor.setPosition(B)
            
    if i == 300*4 :    # Should be above ball pit, drop ball and reset
        gripper.turnOff()
        i = 0
        continue

    i += 1
    pass