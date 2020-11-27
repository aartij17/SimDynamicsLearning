from z3 import *
import math
import json

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

n = 10  # change this to number of iterations later on
theta_d0_sim2 = 0
# theta_d = init_velocity
theta_d1_sim2 = 1  # hard-coded

length_to_mass = 1


def accel(t):
    return -math.sin(t)


def theta_dd():
    return accel(math.pi / 10)


def theta_x(t):
    return 400 + length_to_mass * math.sin(t)


def theta_y(t):
    return 300 + length_to_mass * math.cos(t)


theta_x_list = []
theta_y_list = []

'''
For z3, only simple arithmetic/boolean operations are allowed to act on symbolic variables. 
'''
def recompute_angles_time_step(time_step):
    global theta_d1_sim2, theta_d0_sim2, theta_x_list, theta_y_list
    theta_x_list.append(theta_x(theta_d0_sim2))
    theta_y_list.append(theta_y(theta_d0_sim2))
    theta_d0_sim2 = theta_d0_sim2 + theta_d1_sim2 * time_step
    a_output = accel(theta_d0_sim2)
    sp = float(theta_d1_sim2 + (time_step * a_output))
    return sp, theta_d0_sim2


def get_parameter_errors(motor_damping_proxy, sp, index):
    global theta_d1_sim2
    theta_d1_sim2 = (1 - motor_damping_proxy) * sp
    '''
    sim1 values: regular pybullet simulation 
    sim2 values: reset ang. position and velocity --> everytime step simulation is called, RESET. 
                    --> use values obtained from our equations of motion [this code]
    '''
    # err0 = abs(theta_d0_sim1 - theta_d0_sim2) # error for ang. positions
    err1 = theta_d1_sim2 - theta_d1_sim1_list[
        index]  # error for ang. velocities --> just this if position doesnt pan out
    # err2 = abs(theta_d2_sim1 - theta_d2_sim2) # error for ang. acceleration -->may not wanna consider this
    return err1  # , err1, err2


def solve_for_damping_proxy():
    sp = 0.0
    global theta_d1_sim2
    n = 22000
    for i in range(n):
        if i in time_steps_list:
            sp, theta_d0_sim2 = recompute_angles_time_step(i)
            theta_d0_sim2_list.append(theta_d0_sim2)
    print(theta_d0_sim2_list)
    motor_damping_proxy0 = Real('motor_damping_proxy')
    motor_damping_proxy = motor_damping_proxy0
    # go over each timestep(same `time-step` as that in pybullet), and solve for motor damping proxy
    for index, ts in enumerate(time_steps_list):
        s = Solver()
        s.add(get_parameter_errors(motor_damping_proxy, sp,
                                   index) < 0.5)  # change this to tuple comparison - check in Z3.
        s.check()
        # print(s.statistics()) # --> this is a handy tool to do the final analysis
        m = s.model()
        numerator = int(m[motor_damping_proxy0].as_fraction().numerator)
        denominator = int(m[motor_damping_proxy0].as_fraction().denominator)
        mdp_list.append(float(numerator) / denominator)
        # print("time_step: {}, mdp: {}".format(ts, mdp_float))

# This method is used when we want to collect all the stats after generating the motor damping proxy.
def collect_all_stats():
    f = open("mdp_list.txt")
    lines = f.readlines()
    mdp_list = list(map(lambda x: float(x), lines))
    print(mdp_list)
    n = 22000
    j = 0
    position_velocity_dict = {}
    for i in range(n):
        if i in time_steps_list:
            sp, theta_d0_sim2 = recompute_angles_time_step(i)
            theta_d0_sim2_list.append(theta_d0_sim2)

            theta_d1_sim2 = (1 - mdp_list[j]) * sp
            theta_d1_sim2_list.append(theta_d1_sim2)

            position_velocity_dict[str(i)] = {
                "position": theta_d0_sim2,
                "velocity": theta_d1_sim2,
                "mdp": mdp_list[j]
            }
            j += 1
    f = open("position_velocity_z3_data.txt", "w")
    f.write(json.dumps(position_velocity_dict))



theta_d0_sim1_list = []  # pybullet position
theta_d1_sim1_list = []  # pybullet velocity
theta_d0_sim2_list = []  # z3 position
theta_d1_sim2_list = []  # z3 velocity
time_steps_list = []

mdp_list = []
f = open("position_velocity_pybullet_data.txt", "r")
data_string = f.read()
data_blob = json.loads(data_string)

unsorted_ts = map(lambda x: int(x), data_blob.keys())

for key in sorted(unsorted_ts):
    theta_d0_sim1_list.append(data_blob[str(key)]["position"])
    theta_d1_sim1_list.append(data_blob[str(key)]["velocity"])
    time_steps_list.append(key)



collect_all_stats()
