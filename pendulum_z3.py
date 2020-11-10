from z3 import *
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
  (define θ′′ (accel (/ pi 10)))          ;intial angular acceleration
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

n = 10 # change this to number of iterations later on
theta = 0
#theta_d = init_velocity
theta_d = 1 # hard-coded

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
    global theta_d, theta, theta_x_list, theta_y_list
    theta_x_list.append(theta_x(theta))
    theta_y_list.append(theta_y(theta))
    theta = theta + theta_d * time_step
    a_output = accel(theta)
    sp = float(theta_d + (time_step * a_output))
    return sp, theta_d

def get_parameter_errors(motor_damping_proxy, sp):
    global theta_d
    theta_d1_sim2 = (1 - motor_damping_proxy[0]) * sp
    '''
    sim1 values: regular pybullet simulation 
    sim2 values: reset ang. position and velocity --> everytime step simulation is called, RESET. 
                    --> use values obtained from our equations of motion [this code]
    '''
    theta_d0_sim1_list = [all values form pybullet at time steps]
    theta_d0_sim2_list = [all values form pybullet at time steps]
    err0 = abs(theta_d0_sim1 - theta_d0_sim2) # error for ang. positions
    err1 = abs(theta_d1_sim1 - theta_d1_sim2) # error for ang. velocities --> just this if position doesnt pan out
    err2 = abs(theta_d2_sim1 - theta_d2_sim2) # error for ang. acceleration -->may not wanna consider this
    return err0, err1, err2

def solve_for_damping_proxy():
    sp = 0.0
    global theta_d
    for i in range(n):
        sp, theta_d0_sim1 = recompute_angles_time_step(i)
        theta_d0_sim2_list.append(theta_d0_sim1)

    motor_damping_proxy = Reals('motor_damping_proxy')
     # i don't know what constraint to put here, ask Roman

    solve(get_parameter_errors(motor_damping_proxy, sp, index) < 0.5) # change this to tuple comparision - check in Z3.
    print(theta_x_list)
    print(theta_y_list)

solve_for_damping_proxy()
# def recompute_angles(motor_damping_proxy):
#     global theta_d, theta
#     theta = theta + theta_d * time_step
#     a_output = accel(theta)
#     theta_d = motor_damping_proxy * (theta_d + (time_step * a_output))
#     return theta, theta_d

# def recompute_angles_new(motor_damping_proxy):
#     global theta_d, theta
#     theta = theta + theta_d * time_step
#     a_output = accel(theta)
#     second_part = float(theta_d + (time_step * a_output))
#     print(theta_d)
#     theta_d = motor_damping_proxy[0] * second_part
#     return theta_d


