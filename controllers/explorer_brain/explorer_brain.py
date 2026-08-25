
from controller import Robot, Keyboard
from controller import Robot, Keyboard
import cv2
import numpy as np

# Initialize robot and timestep
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Enable keyboard input
keyboard = Keyboard()
keyboard.enable(timestep)

# Initialize motors
wheelNames = ['motorFrontLeft', 'motorFrontRight', 'motorBackLeft', 'motorBackRight']
wheels = []

for name in wheelNames:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    wheels.append(motor)

# Omnidirectional (Mecanum) kinematics
def move(vx, vy, omega):
    # Calculate individual wheel speeds
    speeds = [
        vy - vx - omega,  # Front Left
        vy + vx + omega,  # Front Right
        vy + vx - omega,  # Back Left
        vy - vx + omega   # Back Right
    ]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])

# Initialize ultrasonic sensor
distanceSensor = robot.getDevice('ultrasonicFront')
distanceSensor.enable(timestep)

# Initialize main camera
cameraMain = robot.getDevice('cameraMain')
cameraMain.enable(timestep)

cameraWidth = cameraMain.getWidth()
cameraHeight = cameraMain.getHeight()

baseSpeed = 4.0

# Main control loop
while robot.step(timestep) != -1:
    key = keyboard.getKey()
    
    # Read sensor value
    sensorValue = distanceSensor.getValue()
    
    vx = 0.0
    vy = 0.0
    omega = 0.0
    
    # Keyboard controls (WASD for translation, QE for rotation)
    if key == ord('W'):
        vy = baseSpeed
    elif key == ord('S'):
        vy = -baseSpeed
    elif key == ord('A'):
        vx = -baseSpeed
    elif key == ord('D'):
        vx = baseSpeed
    elif key == ord('Q'):
        omega = baseSpeed
    elif key == ord('E'):
        omega = -baseSpeed
        
    # Read raw camera image
    imageRaw = cameraMain.getImage()
    
    # Convert raw image to OpenCV format (NumPy array)
    if imageRaw:
        # Webots returns BGRA format by default
        imageArray = np.frombuffer(imageRaw, np.uint8).reshape((cameraHeight, cameraWidth, 4))
        
        # Display the image in a separate OpenCV window
        cv2.imshow("OmniVisionBot Camera Feed", imageArray)
        cv2.waitKey(1)
        
    # Emergency brake system (Frontal collision avoidance)
    if sensorValue < 0.25 and vy > 0:
        vy = 0.0
        print("--- Emergency Brake Activated! Obstacle detected. ---")
        
    # Apply velocities to wheels
    move(vx, vy, omega)