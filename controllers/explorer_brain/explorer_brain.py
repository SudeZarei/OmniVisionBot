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


def move(vx, vy, omega):
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

baseSpeed = 10.0

# ---------------------------------------------------------------------------
# ANALYTICAL IPM CALIBRATION (Option C)
# ---------------------------------------------------------------------------
CAMERA_HEIGHT_M = 0.05
CAMERA_PITCH_RAD = 0
Z_NEAR_M = 0.08
Z_FAR_M = 0.30
HALF_WIDTH_M = 0.12

hFov = cameraMain.getFov()
vFov = 2 * np.arctan(np.tan(hFov / 2) * (cameraHeight / cameraWidth))
fx = (cameraWidth / 2) / np.tan(hFov / 2)
fy = (cameraHeight / 2) / np.tan(vFov / 2)
cx, cy = cameraWidth / 2, cameraHeight / 2


def project_ground_point(X, Z, height, pitch):
    x_c0 = X
    y_c0 = height
    z_c0 = Z

    y_c = y_c0 * np.cos(pitch) - z_c0 * np.sin(pitch)
    z_c = y_c0 * np.sin(pitch) + z_c0 * np.cos(pitch)
    x_c = x_c0

    if z_c <= 1e-6:
        return None

    u = fx * (x_c / z_c) + cx
    v = fy * (y_c / z_c) + cy
    return (u, v)


worldCorners = [
    (-HALF_WIDTH_M, Z_NEAR_M),
    (HALF_WIDTH_M, Z_NEAR_M),
    (HALF_WIDTH_M, Z_FAR_M),
    (-HALF_WIDTH_M, Z_FAR_M),
]

ptsSrcList = []
for (X, Z) in worldCorners:
    p = project_ground_point(X, Z, CAMERA_HEIGHT_M, CAMERA_PITCH_RAD)
    if p is None:
        raise ValueError("World point projects behind/at the camera.")
    ptsSrcList.append(p)

ptsSrc = np.float32(ptsSrcList)
ptsDst = np.float32([
    [0, cameraHeight],
    [cameraWidth, cameraHeight],
    [cameraWidth, 0],
    [0, 0]
])

matrix = cv2.getPerspectiveTransform(ptsSrc, ptsDst)

traffic_state = "GREEN"
yellow_memory_timer = 0
# ---------------------------------------------------------------------------

# Main control loop
while robot.step(timestep) != -1:
    key = keyboard.getKey()
    sensorValue = distanceSensor.getValue()

    imageRaw = cameraMain.getImage()

    vx = 0.0
    vy = 0.0
    omega = 0.0

    if imageRaw:
        imageArray = np.frombuffer(imageRaw, np.uint8).reshape((cameraHeight, cameraWidth, 4)).copy()
        gray = cv2.cvtColor(imageArray, cv2.COLOR_BGRA2GRAY)
        
        # Color Detection System (HSV)
        bgr = cv2.cvtColor(imageArray, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        mask_red1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        mask_yellow = cv2.inRange(hsv, np.array([15, 150, 150]), np.array([35, 255, 255]))
        mask_green = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([85, 255, 255]))
        
        pixels_red = cv2.countNonZero(mask_red)
        pixels_yellow = cv2.countNonZero(mask_yellow)
        pixels_green = cv2.countNonZero(mask_green)
        
        detection_threshold = 30
        
        # Update traffic state based on current vision
        if pixels_red > detection_threshold:
            traffic_state = "RED"
        elif pixels_green > detection_threshold:
            traffic_state = "GREEN"
            yellow_memory_timer = 0
        elif pixels_yellow > detection_threshold:
            traffic_state = "YELLOW"
            yellow_memory_timer = 70

        # Perspective Transform for Line Following
        warped = cv2.warpPerspective(gray, matrix, (cameraWidth, cameraHeight), borderValue=255)

        cropTop = int(cameraHeight * 0.5)
        roi = warped[cropTop:cameraHeight, 0:cameraWidth]

        _, thresh = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx_pt = int(M["m10"] / M["m00"])
                cy_pt = int(M["m01"] / M["m00"]) + cropTop

                cv2.circle(warped, (cx_pt, cy_pt), 5, (0, 0, 255), -1)

                error = cx_pt - (cameraWidth / 2)
                kp = 0.008

               # 1. Base logic for normal speed and steering
                vy = baseSpeed * 0.7
                omega = -error * kp
                
                # 2. Apply Traffic Light Logic (Overrides Base Speed)
                if traffic_state == "RED":
                    vy = 0.0
                    omega = 0.0
                    
                elif traffic_state == "YELLOW":
                    vy = baseSpeed * 0.3  # سرعت کند
                    
               
                    if pixels_yellow <= detection_threshold:
                        yellow_memory_timer -= 1
                        
                 
                    if yellow_memory_timer <= 0:
                        traffic_state = "GREEN"
                
        cv2.imshow("OmniVisionBot Camera Feed", imageArray)
        cv2.imshow("Warped (Birds-Eye)", warped)
        cv2.imshow("Brain View (Focused ROI)", thresh)
        cv2.waitKey(1)

    # Keyboard override (works regardless of traffic lights)
    if key == ord('W'):
        vy = baseSpeed
        omega = 0.0
    elif key == ord('S'):
        vy = -baseSpeed
        omega = 0.0
    elif key == ord('A'):
        vx = -baseSpeed
        omega = 0.0
    elif key == ord('D'):
        vx = baseSpeed
        omega = 0.0
    elif key == ord('Q'):
        omega = baseSpeed
        vy = 0.0
    elif key == ord('E'):
        omega = -baseSpeed
        vy = 0.0

    if sensorValue < 0.25 and vy > 0:
        vy = 0.0
        print("--- Emergency Brake Activated! ---")

    move(vx, vy, omega)