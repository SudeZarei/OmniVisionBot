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

baseSpeed = 4.0

# ---------------------------------------------------------------------------
# ANALYTICAL IPM CALIBRATION (Option C)
# ---------------------------------------------------------------------------
# Fill these in from your Webots model - no manual pixel-clicking needed.

# Physical height of the camera above the ground plane, in METERS.
# Read this from the camera Transform's translation field in your .wbt /
# proto (its Y or Z coordinate, depending on your world's up-axis),
# possibly summed with the robot base's own height off the floor.
CAMERA_HEIGHT_M = 0.05  # <-- REPLACE with your real value

# Camera pitch relative to horizontal, in radians. 0.0 for the level
# ("not_rotate") mounting you're using now.
CAMERA_PITCH_RAD = 0

# The real-world rectangle on the ground, directly ahead of the robot,
# that you want to "unwarp" into a rectangle. Tune these to taste:
#   Z_NEAR_M / Z_FAR_M -> how close / how far ahead to look (meters)
#   HALF_WIDTH_M       -> half-width of the corridor to capture (meters)
Z_NEAR_M = 0.08                 # <-- tune
Z_FAR_M = 0.30                  # <-- tune
HALF_WIDTH_M = 0.12             # <-- tune

# --- Intrinsics from Webots FOV (no change needed) ---
hFov = cameraMain.getFov()
vFov = 2 * np.arctan(np.tan(hFov / 2) * (cameraHeight / cameraWidth))
fx = (cameraWidth / 2) / np.tan(hFov / 2)
fy = (cameraHeight / 2) / np.tan(vFov / 2)
cx, cy = cameraWidth / 2, cameraHeight / 2


def project_ground_point(X, Z, height, pitch):
    """
    Project a ground-plane world point (lateral X, forward Z, both meters,
    camera at given height/pitch) into pixel coordinates (u, v).
    World frame: X right, Z forward, ground at Y=0, camera at (0, height, 0).
    """
    # vector from camera to point, expressed in the "un-tilted" camera axes
    x_c0 = X
    y_c0 = height
    z_c0 = Z

    # apply pitch rotation about the camera's x-axis
    y_c = y_c0 * np.cos(pitch) - z_c0 * np.sin(pitch)
    z_c = y_c0 * np.sin(pitch) + z_c0 * np.cos(pitch)
    x_c = x_c0

    if z_c <= 1e-6:
        return None  # behind the camera / at the horizon - not visible

    u = fx * (x_c / z_c) + cx
    v = fy * (y_c / z_c) + cy
    return (u, v)


# Corners of the chosen real-world rectangle, in the SAME order used
# throughout (bottom-left, bottom-right, top-right, top-left) - "bottom"
# = near, "top" = far, matching image row direction.
worldCorners = [
    (-HALF_WIDTH_M, Z_NEAR_M),  # bottom-left  (near, left)
    (HALF_WIDTH_M, Z_NEAR_M),   # bottom-right (near, right)
    (HALF_WIDTH_M, Z_FAR_M),    # top-right    (far, right)
    (-HALF_WIDTH_M, Z_FAR_M),   # top-left     (far, left)
]

ptsSrcList = []
for (X, Z) in worldCorners:
    p = project_ground_point(X, Z, CAMERA_HEIGHT_M, CAMERA_PITCH_RAD)
    if p is None:
        raise ValueError(
            f"World point (X={X}, Z={Z}) projects behind/at the camera - "
            f"reduce Z_FAR_M or check CAMERA_HEIGHT_M/CAMERA_PITCH_RAD."
        )
    ptsSrcList.append(p)

ptsSrc = np.float32(ptsSrcList)
print("Computed ptsSrc (analytical IPM):", ptsSrc)

ptsDst = np.float32([
    [0, cameraHeight],
    [cameraWidth, cameraHeight],
    [cameraWidth, 0],
    [0, 0]
])

matrix = cv2.getPerspectiveTransform(ptsSrc, ptsDst)
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

        warped = cv2.warpPerspective(gray, matrix, (cameraWidth, cameraHeight),
                                      borderValue=255)

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

                vy = baseSpeed * 1.2
                omega = -error * kp
                
        cv2.imshow("OmniVisionBot Camera Feed", imageArray)
        cv2.imshow("Warped (Birds-Eye)", warped)
        cv2.imshow("Brain View (Focused ROI)", thresh)
        cv2.waitKey(1)

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