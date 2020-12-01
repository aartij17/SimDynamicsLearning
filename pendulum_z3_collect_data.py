import json
import math

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
z3_position = 0
# theta_d = init_velocity
z3_velocity = 1  # hard-coded

length_to_mass = 1


def accel(t):
    return -math.sin(t)


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


def recompute_angles_time_step(time_step):
    global z3_velocity, z3_position, z3_position_x_list, z3_position_y_list
    z3_position_x_list.append(z3_position_x(z3_position))
    z3_position_y_list.append(z3_position_y(z3_position))
    z3_position = z3_position + z3_velocity * time_step
    a_output = accel(z3_position)
    sp = float(z3_velocity + (time_step * a_output))
    return sp, z3_position


def get_parameter_errors(motor_damping_proxy, sp, index):
    global z3_velocity
    z3_velocity = (1 - motor_damping_proxy) * sp
    '''
    sim1 values: regular pybullet simulation 
    sim2 values: reset ang. position and velocity --> everytime step simulation is called, RESET. 
                    --> use values obtained from our equations of motion [this code]
    '''
    # err0 = abs(pb_position - z3_position) # error for ang. positions
    err1 = z3_velocity - pb_velocity_list[
        index]  # error for ang. velocities --> just this if position doesnt pan out
    # err2 = abs(theta_d2_sim1 - theta_d2_sim2) # error for ang. acceleration -->may not wanna consider this
    return err1  # , err1, err2


# This method is used when we want to collect all the stats after generating the motor damping proxy.
def collect_all_stats():
    f = open("mdp_list.txt")
    lines = f.readlines()
    mdp_list = list(map(lambda x: float(x), lines))
    print(mdp_list)
    n = config["time_steps"]
    j = 0
    position_velocity_dict = {}
    for i in range(n):
        if i in time_steps_list:
            sp, z3_position = recompute_angles_time_step(i)
            z3_position_list.append(z3_position)

            z3_velocity = (1 - mdp_list[j]) * sp
            z3_velocity_list.append(z3_velocity)

            position_velocity_dict[str(i)] = {
                "position": z3_position,
                "velocity": z3_velocity,
                "mdp": mdp_list[j]
            }
            j += 1
    f = open("position_velocity_z3_data.txt", "w")
    f.write(json.dumps(position_velocity_dict))


pb_position_list = []  # pybullet position
pb_velocity_list = []  # pybullet velocity
z3_position_list = []  # z3 position
z3_velocity_list = []  # z3 velocity
time_steps_list = []

mdp_list = []
f = open("position_velocity_pybullet_data.txt", "r")
data_string = f.read()
data_blob = json.loads(data_string)

unsorted_ts = map(lambda x: int(x), data_blob.keys())

config_file = open("config.json")
config = json.load(config_file)

for key in sorted(unsorted_ts):
    pb_position_list.append(data_blob[str(key)]["position"])
    pb_velocity_list.append(data_blob[str(key)]["velocity"])
    time_steps_list.append(key)

collect_all_stats()
