from math import sqrt, atan2, pi
# from platform import uname
# from Integrator import Integrator
from Odometry import Odometry
import time
from ev3dev2 import motor, sound
spkr = sound.Sound()

f = None
TARGETS = [(0.5, 0), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]

# Robot config
# WHEEL_RADIUS = 3.4 / 100  # wheel radius (meters)
# BASE =  11.2  / 100  # space between centers of wheels!!! (meters)
# WHEEL_RADIUS = (9.4/2) / 100  # wheel radius (meters)
# BASE =  13  / 100  # space between centers of wheels!!! (meters)
WHEEL_RADIUS = (7/2) / 100  # wheel radius (meters)
BASE =  13.2 / 100  # space between centers of wheels!!! (meters)
T = 0.045  # cycle time



# Reg parameters
KS = 80  # linear speed
KR = 40 # angular speed
U_MAX = 75  # max duty_cycle_sp
U_MIN = 20 # min duty_cycle_sp

# f = open("map_ks{}_kr{}_umax{}_umin{}.csv".format(KS, KR, U_MAX, U_MIN), "w+")
# f.write("x, y, ul, ur, rho, alpha, theta\n")
# Motors init
L_MOTOR = motor.LargeMotor(motor.OUTPUT_A)
R_MOTOR = motor.LargeMotor(motor.OUTPUT_B)

# Odometry init
OD = Odometry(WHEEL_RADIUS, BASE, T)

# wheel_radius =  2.5/100
# base = 15/100
# B = 15cm
# R  = 2.5cm

# def normalize_angle(angle):
#     while angle > pi:
#         angle -= 2*pi
#     while angle < -pi:
#         angle += 2*pi
#     return angle

def normalize_angle(angle):
    if angle >= pi:
        return angle - 2*pi
    elif angle <= -pi:
        return angle + 2*pi
    else: return angle

# alpha to right - negative, to left - positive
def get_error(xg, yg, x, y, theta) -> tuple:
    error_x = xg - x
    error_y = yg - y
    rho = sqrt(error_x ** 2 + error_y ** 2)
    alpha = normalize_angle(atan2(error_y, error_x) - theta)
    return (rho, alpha)

def saturation(u, u_max=U_MAX, u_min=U_MIN):
    if u > u_max:
        return u_max
    elif u < -u_max:
        return -u_max
    elif 0 < u < u_min: return u_min
    elif -u_min < u < 0: return -u_min
    else: return u

def control(x_goal: float, y_goal: float):
    print("STAAART")
    state = "TURN"
    while True:
        # spkr.speek("turn")
        t1 = time.time()

        # 1. Update coordinates
        x, y, theta = OD.update(
            L_MOTOR.speed * pi/180,
            R_MOTOR.speed * pi/180
        )
        
        # 2. Calculate error
        speed_error, angular_error = get_error(x_goal, y_goal, x, y, theta)
        # print("X: {}, Y: {}, Rho: {} alpha: {}".format(x, y, speed_error, angular_error))

        # 3. Checkout stop-condition
        
        if round(speed_error, 2) < 0.05:
            L_MOTOR.stop()
            R_MOTOR.stop()
            print("Target {} {} reached!".format(x_goal, y_goal))
            break
        if round(angular_error, 3) < 0.005:
            state = "FORWARD"
            # time.sleep(0.5)
            # spkr.speek("forward")

        # 4. Расчет управляющих сигналов
        v_g = saturation(KS * speed_error)
        w_g = saturation(KR * angular_error, u_max=40, u_min=5)

        if state == "TURN":
            ul = saturation(-w_g, u_min=18, u_max=35)
            ur = saturation(w_g, u_min=18, u_max=35)
        if state == "FORWARD":
            ul = saturation(v_g - w_g, u_max=60)
            ur = saturation(v_g + w_g, u_max=60)
        
        # print("")
        print("ul: {}, ur: {}, rho: {}, alpha: {}".format(ul, ur, speed_error, angular_error))

        if f is not None:
            f.write('{}, {}, {}, {}, {}, {}, {}\n'.format(x, y, ul, ur, speed_error, angular_error, theta))

        # 7. Motors management
        L_MOTOR.run_direct(duty_cycle_sp=ul)
        R_MOTOR.run_direct(duty_cycle_sp=ur)

        # 8. Pause for next iteration
        dt = T - (time.time() - t1)
        if dt > 0:
            time.sleep(dt)
        else:
            print("warning")



if __name__ == "__main__":
    try:
        #f = open("map", "w+")
        # spkr.play_file("lets-go-cat.mp3")
        spkr.speak("start")
        for x, y in TARGETS:
            print("Current target: ({}, {})".format(x, y))
            control(x, y)
            spkr.speak("task {} {} done".format(x, y), volume=100)
        # # Setup target
        # TARGET_X = 1
        # TARGET_Y = 1

        # # print("Starting...")
        # main_loop(TARGET_X, TARGET_Y)

    # except Exception as e:
    #     # print("Ошибка:", e)
    finally:
        L_MOTOR.stop()
        R_MOTOR.stop()
        spkr.speak("all tasks done")
