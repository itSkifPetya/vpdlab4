from math import sqrt, atan2, pi
from Odometry import Odometry
import time
from ev3dev2 import motor, sound

f = None
# Speaker initialization
spkr = sound.Sound()

# Targets
# TARGETS = [(0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5), (0.5, 0.5)]
TARGETS = [(0.5, 0), (0, 0.5), (-0.5, 0), (0, -0.5)]

### Robot configs
# WHEEL_RADIUS = 3.4 / 100  # wheel radius (meters)
# BASE =  11.2  / 100  # space between centers of wheels!!! (meters)

# WHEEL_RADIUS = (9.4/2) / 100  # wheel radius (meters)
# BASE =  13  / 100  # space between centers of wheels!!! (meters)

WHEEL_RADIUS = (7/2) / 100  # wheel radius (meters)
# BASE =  11.5 / 100  # space between centers of wheels!!! (meters)
BASE =  12 / 100  # space between centers of wheels!!! (meters)

# Cycle period
T = 0.045  

# Reg params
KS = 120  # linear speed
KR = 160 # angular speed

# Default saturation params
U_MAX = 90  # max duty_cycle_sp
U_MIN = 25 # min duty_cycle_sp



# Motors init
L_MOTOR = motor.LargeMotor(motor.OUTPUT_C)
R_MOTOR = motor.LargeMotor(motor.OUTPUT_B)

# Odometry init
# OD = Odometry(WHEEL_RADIUS, BASE, T)

# Normalize angle function
def normalize_angle(angle):
    if angle >= pi:
        return angle - 2*pi
    elif angle <= -pi:
        return angle + 2*pi
    else: return angle

# alpha: to right - negative, to left - positive

# Error calculation function
def get_error(xg, yg, x, y, theta) -> tuple:
    error_x = xg - x
    error_y = yg - y
    rho = sqrt(error_x ** 2 + error_y ** 2)
    alpha = normalize_angle(atan2(error_y, error_x) - theta)
    return (rho, alpha)

# Extended saturation function sets high and low limits
def saturation(u, u_max=U_MAX, u_min=U_MIN):
    if u > u_max:
        return u_max
    elif u < -u_max:
        return -u_max
    elif 0 < u < u_min: return u_min
    elif -u_min < u < 0: return -u_min
    else: return u

# Main control function
def control(x_goal: float, y_goal: float, temp, odometry):
    while True:
        cycle_start_time = time.time()
        # Update coordinates
        x, y, theta = odometry.update(
            L_MOTOR.speed * pi/180,
            R_MOTOR.speed * pi/180
        )
        print(x, y)
        # Calculate control
        speed_error, angular_error = get_error(x_goal, y_goal, x, y, theta)
        
        # Destination check
        if round(speed_error, 2) < 0.05:
            # L_MOTOR.stop()
            # R_MOTOR.stop()
            print("Target {} {} reached!".format(x_goal, y_goal))
            break

        # Calculate control
        v_g = saturation(KS * speed_error)
        w_g = saturation(KR * angular_error, u_min=5)

        ul = saturation(v_g - w_g)
        ur = saturation(v_g + w_g)

        # Using temporary string because writing directly to a file every time
        # disturbs period of cycle
        # temp += '{}, {}, {}, {}, {}, {}, {}\n'.format(x, y, ul, ur, speed_error, angular_error, theta)
        temp.append([x, y, ul, ur, speed_error, angular_error, theta])

        # Motors control
        L_MOTOR.run_direct(duty_cycle_sp=ul)
        R_MOTOR.run_direct(duty_cycle_sp=ur)

        # Pause for next iteration
        dt = T - (time.time() - cycle_start_time)
        if dt > 0:
            time.sleep(dt)
        else:
            # warn means that period T is too small
            print("warning")

def data_form(ar) -> str:
    return "\n".join(str(",".join([str(y) for y in x])) for x in ar)
c = 0
if __name__ == "__main__":
    try:
        # spkr.speak("start", volume=50)
        for x_goal, y_goal in TARGETS:
            c+=1
            OD = Odometry(WHEEL_RADIUS, BASE, T)
            # Telemetry file init
            f = open("split_basic_ks{}_kr{}_umax{}_umin{}_{}.csv".format(KS, KR, U_MAX, U_MIN, c), "w+")
            f.write("x, y, ul, ur, rho, alpha, theta\n")
            # temp = ""
            temp_ar = []
            print("Current target: ({}, {})".format(x_goal, y_goal))
            control(x_goal, y_goal, temp_ar, OD)
            if f is not None:
                f.write(data_form(temp_ar))
            print("task {} {} done".format(x_goal, y_goal))
            # spkr.speak("task {} {} done".format(x, y), volume=50)
            time.sleep(10)
            # OD.update(0,0)
    finally:
        L_MOTOR.stop()
        R_MOTOR.stop()
        print("all tasks done")
        # spkr.speak("all tasks done", volume=50)
