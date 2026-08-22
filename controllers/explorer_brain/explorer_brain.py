from controller import Robot, Keyboard

# ۱. راه‌اندازی ربات و زمان‌بندی شبیه‌سازی
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ۲. فعال‌سازی کیبورد
keyboard = Keyboard()
keyboard.enable(timestep)

# ۳. راه‌اندازی موتورها با نام‌های جدید
wheel_names = ['motorFrontLeft', 'motorFrontRight', 'motorBackLeft', 'motorBackRight']
wheels = []

for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    wheels.append(motor)

# ۴. تابع سینماتیک حرکت همه‌جهته (مکانوم/امنی)
def move(vx, vy, omega):
    # ترکیب بردارهای سرعت برای ۴ چرخ
    speeds = [
        vy - vx - omega,  # Front Left
        vy + vx + omega,  # Front Right
        vy + vx - omega,  # Back Left
        vy - vx + omega   # Back Right
    ]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])

# ۵. حلقه اصلی شبیه‌سازی
speed = 6.0  # سرعت پایه ربات

while robot.step(timestep) != -1:
    key = keyboard.getKey()
    
    vx = 0.0
    vy = 0.0
    omega = 0.0
    
    # کنترل‌های حرکتی با کیبورد
    if key == ord('W'):
        vy = speed      # جلو
    elif key == ord('S'):
        vy = -speed     # عقب
    elif key == ord('A'):
        vx = speed     # سُر خوردن به چپ
    elif key == ord('D'):
        vx = -speed      # سُر خوردن به راست
    elif key == ord('Q'):
        omega = speed   # چرخش به چپ
    elif key == ord('E'):
        omega = -speed  # چرخش به راست
        
    move(vx, vy, omega)