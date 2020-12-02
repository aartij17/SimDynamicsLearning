import json
import math
from math import *
from z3 import *

'''
;code was modified from    https://rosettacode.org/wiki/Animate_a_pendulum#Racket
;equations of motion        https://www.youtube.com/watch?v=8VJ1CJ55Np0
;second order equations of motion https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/17%3A_Second-Order_Differential_Equations/17.3%3A_Applications_of_Second-Order_Differential_Equations
#lang racket
(require 2htdp/image 2htdp/universe)
 
(define (pendulum   length_to_mass   motor_damping_proxy   time_step   initial_position_ratio_of_pi   initial_velocity )
  (define (accel theta) (- (sin theta)))
  (define θ (* pi initial_position_ratio_of_pi))                      ;initial angular position
  (define θ′ initial_velocity)                            ;initial angular velocity
  (define θ′′ (accel (/ pi 10)))          ;initial angular acceleration
  (define (x θ) (+ 400 (* length_to_mass (sin θ))))  ;defines how to find the x position of the end effetor
  (define (y θ) (+ 300 (* length_to_mass (cos θ))))  ;defines how to find the y position of the end effector
  (λ (n)
    (define p-image (underlay/xy (add-line (empty-scene 800 600) 400 300 (x θ) (y θ) "black") 
                                 (- (x θ) 5) (- (y θ) 5) (circle 5 "solid" "blue")))
    (set! θ (+ θ (* θ′ time_step )))
    (set! θ′ (* (- 1 motor_damping_proxy) (+ θ′ (* (accel θ) time_step   ) ) ) )
    p-image))
 
(animate (pendulum   150   0.01   (/ 1 10)   1.1   -3 ))
'''

z3_position = 15
z3_velocity = 1  # hard-coded

length_to_mass = 1
TS = float(1.0/240)

def accel(t):
    #y = 0
    # for k in range(0,5,1):
    #     y+=((-1)**k)*(t**(1+2*k))/factorial(1+2*k)
    # return y
    #return -((16 * t) * (math.pi - t))/((5 * math.pow(math.pi, 2)) - (4 * t * (math.pi - t)))
    return math.sin(t)


def theta_dd():
    return accel(math.pi / 10)


def z3_position_x(t):
    return 400 + length_to_mass * math.sin(t)


def z3_position_y(t):
    return 300 + length_to_mass * math.cos(t)


z3_position_x_list = []
z3_position_y_list = []

'''
For z3, only simple arithmetic/boolean operations are allowed to act on symbolic variables. 
'''


# TODO: Recheck validity wrt equations of motion
def recompute_angles_time_step(time_step):
    global z3_velocity, z3_position, z3_position_x_list, z3_position_y_list, z3_position_old
    z3_position_x_list.append(z3_position_x(z3_position))
    z3_position_y_list.append(z3_position_y(z3_position))
    z3_position_old = z3_position
    z3_position = z3_position + z3_velocity * TS
    a_output = accel(z3_position)
    sp = float(z3_velocity + (TS * a_output))
    return sp, z3_position

def get_velocity_errors(motor_damping_proxy, sp, index, ts):
    global z3_velocity
    z3_velocity_local = (1 - motor_damping_proxy) * sp
    err1 = z3_velocity_local - pb_velocity_list[index]
    return err1

def get_position_errors(index):
    global z3_position
    return z3_position%(2*math.pi) - pb_position_list[index]%(2*math.pi)

position_velocity_dict = {}

def solve_for_damping_proxy():
    global z3_velocity, z3_position, z3_position_old
    position_error = config["position_error"]
    velocity_error = config["velocity_error"]
    for index, ts in enumerate(time_steps_list):
        sp, z3_position = recompute_angles_time_step(ts)
        motor_damping_proxy0 = Real('motor_damping_proxy')
        motor_damping_proxy = motor_damping_proxy0
        s = Solver()
        # position_error /= 1.05
        # velocity_error /= 1.05
        # position_error = config["position_error"]
        # velocity_error = config["velocity_error"]
        # if index == 0: # loosen the constraint
        #     position_error = config["position_error"] / config["constraint_scale"]
        #     velocity_error = config["velocity_error"] / config["constraint_scale"]
        # if index == len(time_steps_list)/2: # tighten the constraint
        #     position_error = config["position_error"] / config["constraint_scale"]
        #     velocity_error = config["velocity_error"] / config["constraint_scale"]
        # else:
        #     position_error = config["position_error"]
        #     velocity_error = config["velocity_error"]

        s.add(And(get_velocity_errors(motor_damping_proxy, sp,index, ts) < velocity_error,
                  get_velocity_errors(motor_damping_proxy, sp,index, ts) > -velocity_error,
                  get_position_errors(index) < position_error,
                  get_position_errors(index) > -position_error))

        print(s.check())
        m = s.model()
        numerator = int(m[motor_damping_proxy0].as_fraction().numerator)
        denominator = int(m[motor_damping_proxy0].as_fraction().denominator)
        mdp_list.append(float(numerator) / denominator)

        z3_velocity = (1 - (sum(mdp_list)/len(mdp_list))) * sp
        z3_position = z3_position + (z3_velocity * TS)
        #z3_velocity = (1 - mdp_list[-1]) * sp

        a_output = accel(z3_position)
        sp = float(z3_velocity + (TS * a_output))
        z3_velocity = (1 - mdp_list[-1]) * sp
        z3_position = (z3_position + (z3_velocity * TS))# % (2*math.pi)
        print("timestamp: {}, z3_position: {}, z3_velocity: {}, sp: {}"
              .format(ts, z3_position, z3_velocity, sp))

        position_velocity_dict[str(ts)] = {
            "position": z3_position_old,
            "velocity": z3_velocity,
            "mdp": mdp_list[-1]
        }

    f = open("position_velocity_z3_data.txt", "w")
    f.write(json.dumps(position_velocity_dict))

def solve_mdp_analysis(initial_position, initial_velocity):
    global z3_position, z3_velocity, mdp_list
    z3_position =  initial_position
    z3_velocity = initial_velocity

    position_error = config["position_error"]
    velocity_error = config["velocity_error"]
    print("IN MDP DEF!, init_pos: {}, init_velocity: {}".format(initial_position, initial_velocity))
    for index, ts in enumerate(time_steps_list):
        sp, z3_position = recompute_angles_time_step(ts)
        motor_damping_proxy0 = Real('motor_damping_proxy')
        motor_damping_proxy = motor_damping_proxy0
        s = Solver()
        s.add(And(get_velocity_errors(motor_damping_proxy, sp,index, ts) < velocity_error,
                  get_velocity_errors(motor_damping_proxy, sp,index, ts) > -velocity_error,
                  get_position_errors(index) < position_error,
                  get_position_errors(index) > -position_error))

        if s.check() != z3.sat:
            if len(mdp_list) == 0:
                print("Empty mdp!")
                return 0
            print("mdp_before_return:{}".format(mdp_list))
            mdp_list_last = mdp_list[-1]
            #mdp_list = []
            return mdp_list_last#sum(mdp_list)/len(mdp_list)

        m = s.model()
        numerator = int(m[motor_damping_proxy0].as_fraction().numerator)
        denominator = int(m[motor_damping_proxy0].as_fraction().denominator)
        if numerator != 0:
            mdp_list.append(float(numerator) / float(denominator))
        else:
            return -1000000

        z3_velocity = (1 - (sum(mdp_list)/len(mdp_list))) * sp
        z3_position = z3_position + (z3_velocity * TS)

        a_output = accel(z3_position)
        sp = float(z3_velocity + (TS * a_output))
        z3_velocity = (1 - mdp_list[-1]) * sp
        z3_position = (z3_position + (z3_velocity * TS))# % (2*math.pi)
        print("timestamp: {}, z3_position: {}, z3_velocity: {}, sp: {}"
              .format(ts, z3_position, z3_velocity, sp))

    print("mdp_list: {}".format(mdp_list))
    average_mdp = mdp_list[-1]#sum(mdp_list) / len(mdp_list)
    #mdp_list = []
    return average_mdp


def init_synthesize():
    global pb_position_list, pb_velocity_list, time_steps_list, mdp_list
    f = open("position_velocity_pybullet_data.txt", "r")
    data_string = f.read()
    data_blob = json.loads(data_string)
    unsorted_ts = map(lambda x: int(x), data_blob.keys())
    for key in sorted(unsorted_ts):
        pb_position_list.append(data_blob[str(key)]["position"])
        pb_velocity_list.append(data_blob[str(key)]["velocity"])
        time_steps_list.append(key)
    mdp_list = []


pb_position_list = []  # pybullet position
pb_velocity_list = []  # pybullet velocity
z3_position_list = []  # z3 position
z3_velocity_list = []  # z3 velocity
time_steps_list = []

mdp_list = []

config_file = open("config.json")
config = json.load(config_file)



#solve_for_damping_proxy()
with open("mdp_list.txt", "w") as f:
    for i in mdp_list:
        f.write(str(i))
        f.write("\n")
