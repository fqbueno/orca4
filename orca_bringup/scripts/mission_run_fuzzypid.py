#!/usr/bin/env python3

from enum import Enum

import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import skfuzzy as fuzz
from matplotlib import pyplot as plt
from skfuzzy import control as ctrl
import cv2
import cv_bridge
import queue
import time
import math

bridge = cv_bridge.CvBridge()

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 300 

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 2000

# Robot's speed when following the line
LINEAR_SPEED = 0.08

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
# KP = 1.10
# KI = 2.00
# KD = 0.60
# KS = 10

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.001

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 15

# BGR values to filter only the selected color range
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    return (1*height//3, height, width//4, 3*width//4)

image_input = 0
error_int = 0
error_dif = 0
error_prev = 0
error = 0
max_error_der = 0
max_neg_error_der= 0
# max = 0
max_neg = 0
p_out = 0
i_out = 0
d_out = 0
kP = 1.10
kI = 2.00
kD = 0.60
kS = 10
error_log = []
time_log = []
control_log = []

err_history = queue.Queue(kS)
t_prev = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None

def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    global should_move
    global right_mark_count
    global finalization_countdown
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input'
    """
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # print('testing sa image')
    # node.get_logger().info('Received image')

def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line) 
    and return the side in which the smaller contour is (the track mark) 
    (If there are any of these contours),
    and draw all contours on 'out' image
    """ 
    # get a list of contours
    contours, _ = cv2.findContours(cv2.bitwise_not(mask), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # print(len(contours))

    mark = {}
    line = {}

    for contour in contours:
        
        M = cv2.moments(contour)
        # Search more about Image Moments on Wikipedia :)

        if M['m00'] > MIN_AREA:
        # if countor.area > MIN_AREA:

            if (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of the track
                line['x'] = crop_w_start + int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

                # plot the area in light blue
                cv2.drawContours(out, contour, -1, (255,255,0), 2) 
                cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
            
            else:
                # Contour is a track mark
                if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                    # if there are more than one mark, consider only 
                    # the one closest to the robot 
                    mark['y'] = int(M["m01"]/M["m00"])
                    mark['x'] = crop_w_start + int(M["m10"]/M["m00"])
                    

                    # plot the area in pink
                    cv2.drawContours(out, contour, -1, (255,0,255), 2) 
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)


    if mark and line:
    # if both contours exist
        if mark['x'] > line['x']:
            mark_side = "right"
        else:
            mark_side = "left"
    else:
        mark_side = None


    return (line, mark_side)

def fuzzy_membership():
    global f_error
    global f_errorder
    global p_controller
    global p_simulation
    global i_simulation
    global i_controller
    global d_simulation
    global d_controller

    # f_error = ctrl.Antecedent(np.arange(-0.6, 0.6, 0.012), "error")
    # f_errorder = ctrl.Antecedent(np.arange(-0.4, 0.6, 0.01), "errorder")
    f_error = ctrl.Antecedent(np.arange(-1, 1, 0.02), "error")
    f_errorder = ctrl.Antecedent(np.arange(-1, 1, 0.02), "errorder")
    
    # f_error["NL"] = fuzz.trimf(f_error.universe, [-0.6, -0.6, -0.4])
    # f_error["NM"] = fuzz.trimf(f_error.universe, [-0.6, -0.4, -0.20000000000000004])
    # f_error["NS"] = fuzz.trimf(f_error.universe, [-0.4, -0.20000000000000004, (-5.551115123125783 * (10**(-17)))])
    # f_error["Z"] = fuzz.trimf(f_error.universe, [-0.20000000000000004, (-5.551115123125783 * (10**(-17))), 0.19999999999999993])
    # f_error["PS"] = fuzz.trimf(f_error.universe, [(-5.551115123125783 * (10**(-17))), 0.19999999999999993, 0.3999999999999999])
    # f_error["PM"] = fuzz.trimf(f_error.universe, [0.19999999999999993, 0.3999999999999999, 0.6])
    # f_error["PL"] = fuzz.trimf(f_error.universe, [0.3999999999999999, 0.6, 0.6])

    f_error["NL"] = fuzz.trimf(f_error.universe, [-1, -1, -0.6666666666666667])
    f_error["NM"] = fuzz.trimf(f_error.universe, [-1, -0.6666666666666667, -0.3333333333333334])
    f_error["NS"] = fuzz.trimf(f_error.universe, [-0.6666666666666667, -0.3333333333333334, (-1.1102230246251565 * (10**(-16)))])
    f_error["Z"] = fuzz.trimf(f_error.universe, [-0.3333333333333334, (-1.1102230246251565 * (10**(-16))), 0.3333333333333332])
    f_error["PS"] = fuzz.trimf(f_error.universe, [(-1.1102230246251565 * (10**(-16))), 0.3333333333333332,  0.6666666666666665])
    f_error["PM"] = fuzz.trimf(f_error.universe, [0.3333333333333332, 0.6666666666666665, 1])
    f_error["PL"] = fuzz.trimf(f_error.universe, [0.6666666666666665, 1, 1])

    # f_errorder["NL"] = fuzz.trimf(f_errorder.universe, [-0.4, -0.4, -0.23333333333333336])
    # f_errorder["NM"] = fuzz.trimf(f_errorder.universe, [-0.4, -0.23333333333333336, -0.06666666666666671])
    # f_errorder["NS"] = fuzz.trimf(f_errorder.universe, [-0.23333333333333336, -0.06666666666666671, 0.09999999999999995])
    # f_errorder["Z"] = fuzz.trimf(f_errorder.universe, [-0.06666666666666671, 0.09999999999999995, 0.2666666666666666])
    # f_errorder["PS"] = fuzz.trimf(f_errorder.universe, [0.09999999999999995, 0.2666666666666666, 0.433333333333333324])
    # f_errorder["PM"] = fuzz.trimf(f_errorder.universe, [0.2666666666666666, 0.433333333333333324,0.6])
    # f_errorder["PL"] = fuzz.trimf(f_errorder.universe, [0.433333333333333324, 0.433333333333333324, 0.6])

    f_errorder["NL"] = fuzz.trimf(f_errorder.universe, [-1, -1, -0.6666666666666667])
    f_errorder["NM"] = fuzz.trimf(f_errorder.universe, [-1, -0.6666666666666667, -0.3333333333333334])
    f_errorder["NS"] = fuzz.trimf(f_errorder.universe, [-0.6666666666666667, -0.3333333333333334, (-1.1102230246251565 * (10**(-16)))])
    f_errorder["Z"] = fuzz.trimf(f_errorder.universe, [-0.3333333333333334, (-1.1102230246251565 * (10**(-16))), 0.3333333333333332])
    f_errorder["PS"] = fuzz.trimf(f_errorder.universe, [(-1.1102230246251565 * (10**(-16))), 0.3333333333333332,  0.6666666666666665])
    f_errorder["PM"] = fuzz.trimf(f_errorder.universe, [0.3333333333333332, 0.6666666666666665, 1])
    f_errorder["PL"] = fuzz.trimf(f_errorder.universe, [0.6666666666666665, 1, 1])

    #ProportionalController
    p_controller = ctrl.ControlSystem()
    p_rules = []
    p_simulation = ctrl.ControlSystemSimulation(p_controller)
    p_gain = ctrl.Consequent(np.arange(0, 3, 0.5), "gain")
    # p_gain = ctrl.Consequent(np.arange(-3, 3, 0.5), "gain")

    p_gain["NL"] = fuzz.trimf(p_gain.universe, [0, 0, 0.5])
    p_gain["NM"] = fuzz.trimf(p_gain.universe, [0, 0.5, 1.0])
    p_gain["NS"] = fuzz.trimf(p_gain.universe, [0.5, 1.0, 1.5])
    p_gain["Z"] = fuzz.trimf(p_gain.universe, [1.0, 1.5, 2.0])
    p_gain["PS"] = fuzz.trimf(p_gain.universe, [1.5, 2.0, 2.5])
    p_gain["PM"] = fuzz.trimf(p_gain.universe, [2.0, 2.5, 3])
    p_gain["PL"] = fuzz.trimf(p_gain.universe, [2.5, 3, 3])

    # p_gain["NL"] = fuzz.trimf(p_gain.universe, [-3, -3, -2])
    # p_gain["NM"] = fuzz.trimf(p_gain.universe, [-3, -2, -1.0])
    # p_gain["NS"] = fuzz.trimf(p_gain.universe, [-2, -1.0, 0.0])
    # p_gain["Z"] = fuzz.trimf(p_gain.universe, [-1.0, 0.0, 1.0])
    # p_gain["PS"] = fuzz.trimf(p_gain.universe, [0.0, 1.0, 2.0])
    # p_gain["PM"] = fuzz.trimf(p_gain.universe, [1.0, 2.0, 3])
    # p_gain["PL"] = fuzz.trimf(p_gain.universe, [2.0, 3, 3])

    p_rule1 = ctrl.Rule(f_error["NL"] & f_errorder["NL"], p_gain["PL"])
    p_rule2 = ctrl.Rule(f_error["NL"] & f_errorder["NM"], p_gain["PL"])
    p_rule3 = ctrl.Rule(f_error["NL"] & f_errorder["NS"], p_gain["PM"])
    p_rule4 = ctrl.Rule(f_error["NL"] & f_errorder["Z"], p_gain["PM"])
    p_rule5 = ctrl.Rule(f_error["NL"] & f_errorder["PS"], p_gain["PS"])
    p_rule6 = ctrl.Rule(f_error["NL"] & f_errorder["PM"], p_gain["Z"])
    p_rule7 = ctrl.Rule(f_error["NL"] & f_errorder["PL"], p_gain["Z"])

    p_rule8 = ctrl.Rule(f_error["NM"] & f_errorder["NL"], p_gain["PL"])
    p_rule9 = ctrl.Rule(f_error["NM"] & f_errorder["NM"], p_gain["PL"])
    p_rule10 = ctrl.Rule(f_error["NM"] & f_errorder["NS"], p_gain["PM"])
    p_rule11 = ctrl.Rule(f_error["NM"] & f_errorder["Z"], p_gain["PS"])
    p_rule12 = ctrl.Rule(f_error["NM"] & f_errorder["PS"], p_gain["PS"])
    p_rule13 = ctrl.Rule(f_error["NM"] & f_errorder["PM"], p_gain["Z"])
    p_rule14 = ctrl.Rule(f_error["NM"] & f_errorder["PL"], p_gain["NS"])

    p_rule15 = ctrl.Rule(f_error["NS"] & f_errorder["NL"], p_gain["PM"])
    p_rule16 = ctrl.Rule(f_error["NS"] & f_errorder["NM"], p_gain["PM"])
    p_rule17 = ctrl.Rule(f_error["NS"] & f_errorder["NS"], p_gain["PM"])
    p_rule18 = ctrl.Rule(f_error["NS"] & f_errorder["Z"], p_gain["PS"])
    p_rule19 = ctrl.Rule(f_error["NS"] & f_errorder["PS"], p_gain["Z"])
    p_rule20 = ctrl.Rule(f_error["NS"] & f_errorder["PM"], p_gain["NS"])
    p_rule21 = ctrl.Rule(f_error["NS"] & f_errorder["PL"], p_gain["NS"])

    p_rule22 = ctrl.Rule(f_error["Z"] & f_errorder["NL"], p_gain["PM"])
    p_rule23 = ctrl.Rule(f_error["Z"] & f_errorder["NM"], p_gain["PM"])
    p_rule24 = ctrl.Rule(f_error["Z"] & f_errorder["NS"], p_gain["PS"])
    p_rule25 = ctrl.Rule(f_error["Z"] & f_errorder["Z"], p_gain["Z"])
    p_rule26 = ctrl.Rule(f_error["Z"] & f_errorder["PS"], p_gain["NS"])
    p_rule27 = ctrl.Rule(f_error["Z"] & f_errorder["PM"], p_gain["NM"])
    p_rule28 = ctrl.Rule(f_error["Z"] & f_errorder["PL"], p_gain["NM"])

    p_rule29 = ctrl.Rule(f_error["PS"] & f_errorder["NL"], p_gain["PS"])
    p_rule30 = ctrl.Rule(f_error["PS"] & f_errorder["NM"], p_gain["PS"])
    p_rule31 = ctrl.Rule(f_error["PS"] & f_errorder["NS"], p_gain["Z"])
    p_rule32 = ctrl.Rule(f_error["PS"] & f_errorder["Z"], p_gain["NS"])
    p_rule33 = ctrl.Rule(f_error["PS"] & f_errorder["PS"], p_gain["NS"])
    p_rule34 = ctrl.Rule(f_error["PS"] & f_errorder["PM"], p_gain["NM"])
    p_rule35 = ctrl.Rule(f_error["PS"] & f_errorder["PL"], p_gain["NL"])

    p_rule36 = ctrl.Rule(f_error["PM"] & f_errorder["NL"], p_gain["PS"])
    p_rule37 = ctrl.Rule(f_error["PM"] & f_errorder["NM"], p_gain["Z"])
    p_rule38 = ctrl.Rule(f_error["PM"] & f_errorder["NS"], p_gain["NS"])
    p_rule39 = ctrl.Rule(f_error["PM"] & f_errorder["Z"], p_gain["NM"])
    p_rule40 = ctrl.Rule(f_error["PM"] & f_errorder["PS"], p_gain["NM"])
    p_rule41 = ctrl.Rule(f_error["PM"] & f_errorder["PM"], p_gain["NM"])
    p_rule42 = ctrl.Rule(f_error["PM"] & f_errorder["PL"], p_gain["NL"])

    p_rule43 = ctrl.Rule(f_error["PL"] & f_errorder["NL"], p_gain["Z"])
    p_rule44 = ctrl.Rule(f_error["PL"] & f_errorder["NM"], p_gain["Z"])
    p_rule45 = ctrl.Rule(f_error["PL"] & f_errorder["NS"], p_gain["NM"])
    p_rule46 = ctrl.Rule(f_error["PL"] & f_errorder["Z"], p_gain["NM"])
    p_rule47 = ctrl.Rule(f_error["PL"] & f_errorder["PS"], p_gain["NM"])
    p_rule48 = ctrl.Rule(f_error["PL"] & f_errorder["PM"], p_gain["NL"])
    p_rule49 = ctrl.Rule(f_error["PL"] & f_errorder["PL"], p_gain["NL"])

    p_rules = [p_rule1, p_rule2, p_rule3, p_rule4, p_rule5, p_rule6, p_rule7,
               p_rule8, p_rule9, p_rule10, p_rule11, p_rule12, p_rule13, p_rule14,
               p_rule15, p_rule16, p_rule17, p_rule18, p_rule19, p_rule20, p_rule21,
               p_rule22, p_rule23, p_rule24, p_rule25, p_rule26, p_rule27, p_rule28,
               p_rule29, p_rule30, p_rule31, p_rule32, p_rule33, p_rule34, p_rule35,
               p_rule36, p_rule37, p_rule38, p_rule39, p_rule40, p_rule41, p_rule42,
               p_rule43, p_rule44, p_rule45, p_rule46, p_rule47, p_rule48, p_rule49,]
    
    for rule in p_rules:
        p_controller.addrule(rule)

    # Integral Controller
    i_controller = ctrl.ControlSystem()
    i_rules = []
    i_simulation = ctrl.ControlSystemSimulation(i_controller)
    i_gain = ctrl.Consequent(np.arange(0, 3, 0.5), "gain")
    # i_gain = ctrl.Consequent(np.arange(-3, 3, 0.5), "gain")

    i_gain["NL"] = fuzz.trimf(i_gain.universe, [0, 0, 0.5])
    i_gain["NM"] = fuzz.trimf(i_gain.universe, [0, 0.5, 1.0])
    i_gain["NS"] = fuzz.trimf(i_gain.universe, [0.5, 1.0, 1.5])
    i_gain["Z"] = fuzz.trimf(i_gain.universe, [1.0, 1.5, 2.0])
    i_gain["PS"] = fuzz.trimf(i_gain.universe, [1.5, 2.0, 2.5])
    i_gain["PM"] = fuzz.trimf(i_gain.universe, [2.0, 2.5, 3])
    i_gain["PL"] = fuzz.trimf(i_gain.universe, [2.5, 3, 3])

    # i_gain["NL"] = fuzz.trimf(i_gain.universe, [-3, -3, -2])
    # i_gain["NM"] = fuzz.trimf(i_gain.universe, [-3, -2, -1.0])
    # i_gain["NS"] = fuzz.trimf(i_gain.universe, [-2, -1.0, 0.0])
    # i_gain["Z"] = fuzz.trimf(i_gain.universe, [-1.0, 0.0, 1.0])
    # i_gain["PS"] = fuzz.trimf(i_gain.universe, [0.0, 1.0, 2.0])
    # i_gain["PM"] = fuzz.trimf(i_gain.universe, [1.0, 2.0, 3])
    # i_gain["PL"] = fuzz.trimf(i_gain.universe, [2.0, 3, 3])

    i_rule1 = ctrl.Rule(f_error["NL"] & f_errorder["NL"], i_gain["NL"])
    i_rule2 = ctrl.Rule(f_error["NL"] & f_errorder["NM"], i_gain["NL"])
    i_rule3 = ctrl.Rule(f_error["NL"] & f_errorder["NS"], i_gain["NM"])
    i_rule4 = ctrl.Rule(f_error["NL"] & f_errorder["Z"], i_gain["NM"])
    i_rule5 = ctrl.Rule(f_error["NL"] & f_errorder["PS"], i_gain["NS"])
    i_rule6 = ctrl.Rule(f_error["NL"] & f_errorder["PM"], i_gain["Z"])
    i_rule7 = ctrl.Rule(f_error["NL"] & f_errorder["PL"], i_gain["Z"])

    i_rule8 = ctrl.Rule(f_error["NM"] & f_errorder["NL"], i_gain["NL"])
    i_rule9 = ctrl.Rule(f_error["NM"] & f_errorder["NM"], i_gain["NL"])
    i_rule10 = ctrl.Rule(f_error["NM"] & f_errorder["NS"], i_gain["NM"])
    i_rule11 = ctrl.Rule(f_error["NM"] & f_errorder["Z"], i_gain["NS"])
    i_rule12 = ctrl.Rule(f_error["NM"] & f_errorder["PS"], i_gain["NS"])
    i_rule13 = ctrl.Rule(f_error["NM"] & f_errorder["PM"], i_gain["Z"])
    i_rule14 = ctrl.Rule(f_error["NM"] & f_errorder["PL"], i_gain["Z"])

    i_rule15 = ctrl.Rule(f_error["NS"] & f_errorder["NL"], i_gain["NL"])
    i_rule16 = ctrl.Rule(f_error["NS"] & f_errorder["NM"], i_gain["NM"])
    i_rule17 = ctrl.Rule(f_error["NS"] & f_errorder["NS"], i_gain["NS"])
    i_rule18 = ctrl.Rule(f_error["NS"] & f_errorder["Z"], i_gain["NS"])
    i_rule19 = ctrl.Rule(f_error["NS"] & f_errorder["PS"], i_gain["Z"])
    i_rule20 = ctrl.Rule(f_error["NS"] & f_errorder["PM"], i_gain["PS"])
    i_rule21 = ctrl.Rule(f_error["NS"] & f_errorder["PL"], i_gain["PS"])

    i_rule22 = ctrl.Rule(f_error["Z"] & f_errorder["NL"], i_gain["NM"])
    i_rule23 = ctrl.Rule(f_error["Z"] & f_errorder["NM"], i_gain["NM"])
    i_rule24 = ctrl.Rule(f_error["Z"] & f_errorder["NS"], i_gain["NS"])
    i_rule25 = ctrl.Rule(f_error["Z"] & f_errorder["Z"], i_gain["Z"])
    i_rule26 = ctrl.Rule(f_error["Z"] & f_errorder["PS"], i_gain["PS"])
    i_rule27 = ctrl.Rule(f_error["Z"] & f_errorder["PM"], i_gain["PM"])
    i_rule28 = ctrl.Rule(f_error["Z"] & f_errorder["PL"], i_gain["PM"])

    i_rule29 = ctrl.Rule(f_error["PS"] & f_errorder["NL"], i_gain["NM"])
    i_rule30 = ctrl.Rule(f_error["PS"] & f_errorder["NM"], i_gain["NS"])
    i_rule31 = ctrl.Rule(f_error["PS"] & f_errorder["NS"], i_gain["Z"])
    i_rule32 = ctrl.Rule(f_error["PS"] & f_errorder["Z"], i_gain["PS"])
    i_rule33 = ctrl.Rule(f_error["PS"] & f_errorder["PS"], i_gain["PS"])
    i_rule34 = ctrl.Rule(f_error["PS"] & f_errorder["PM"], i_gain["PM"])
    i_rule35 = ctrl.Rule(f_error["PS"] & f_errorder["PL"], i_gain["PM"])

    i_rule36 = ctrl.Rule(f_error["PM"] & f_errorder["NL"], i_gain["Z"])
    i_rule37 = ctrl.Rule(f_error["PM"] & f_errorder["NM"], i_gain["Z"])
    i_rule38 = ctrl.Rule(f_error["PM"] & f_errorder["NS"], i_gain["PS"])
    i_rule39 = ctrl.Rule(f_error["PM"] & f_errorder["Z"], i_gain["PS"])
    i_rule40 = ctrl.Rule(f_error["PM"] & f_errorder["PS"], i_gain["PM"])
    i_rule41 = ctrl.Rule(f_error["PM"] & f_errorder["PM"], i_gain["PL"])
    i_rule42 = ctrl.Rule(f_error["PM"] & f_errorder["PL"], i_gain["PL"])

    i_rule43 = ctrl.Rule(f_error["PL"] & f_errorder["NL"], i_gain["Z"])
    i_rule44 = ctrl.Rule(f_error["PL"] & f_errorder["NM"], i_gain["Z"])
    i_rule45 = ctrl.Rule(f_error["PL"] & f_errorder["NS"], i_gain["PS"])
    i_rule46 = ctrl.Rule(f_error["PL"] & f_errorder["Z"], i_gain["PS"])
    i_rule47 = ctrl.Rule(f_error["PL"] & f_errorder["PS"], i_gain["PM"])
    i_rule48 = ctrl.Rule(f_error["PL"] & f_errorder["PM"], i_gain["PL"])
    i_rule49 = ctrl.Rule(f_error["PL"] & f_errorder["PL"], i_gain["PL"])

    i_rules = [i_rule1, i_rule2, i_rule3, i_rule4, i_rule5, i_rule6, i_rule7,
               i_rule8, i_rule9, i_rule10, i_rule11, i_rule12, i_rule13, i_rule14,
               i_rule15, i_rule16, i_rule17, i_rule18, i_rule19, i_rule20, i_rule21,
               i_rule22, i_rule23, i_rule24, i_rule25, i_rule26, i_rule27, i_rule28,
               i_rule29, i_rule30, i_rule31, i_rule32, i_rule33, i_rule34, i_rule35,
               i_rule36, i_rule37, i_rule38, i_rule39, i_rule40, i_rule41, i_rule42,
               i_rule43, i_rule44, i_rule45, i_rule46, i_rule47, i_rule48, i_rule49,]
    
    for rule in i_rules:
        i_controller.addrule(rule)

    #Derivative Controller
    d_controller = ctrl.ControlSystem()
    d_rules = []
    d_simulation = ctrl.ControlSystemSimulation(d_controller)
    d_gain = ctrl.Consequent(np.arange(0, 3, 0.5), "gain")
    # d_gain = ctrl.Consequent(np.arange(-3, 3, 0.5), "gain")

    d_gain["NL"] = fuzz.trimf(d_gain.universe, [0, 0, 0.5])
    d_gain["NM"] = fuzz.trimf(d_gain.universe, [0, 0.5, 1.0])
    d_gain["NS"] = fuzz.trimf(d_gain.universe, [0.5, 1.0, 1.5])
    d_gain["Z"] = fuzz.trimf(d_gain.universe, [1.0, 1.5, 2.0])
    d_gain["PS"] = fuzz.trimf(d_gain.universe, [1.5, 2.0, 2.5])
    d_gain["PM"] = fuzz.trimf(d_gain.universe, [2.0, 2.5, 3])
    d_gain["PL"] = fuzz.trimf(d_gain.universe, [2.5, 3, 3])
    # d_gain["NL"] = fuzz.trimf(d_gain.universe, [-3, -3, -2])
    # d_gain["NM"] = fuzz.trimf(d_gain.universe, [-3, -2, -1.0])
    # d_gain["NS"] = fuzz.trimf(d_gain.universe, [-2, -1.0, 0.0])
    # d_gain["Z"] = fuzz.trimf(d_gain.universe, [-1.0, 0.0, 1.0])
    # d_gain["PS"] = fuzz.trimf(d_gain.universe, [0.0, 1.0, 2.0])
    # d_gain["PM"] = fuzz.trimf(d_gain.universe, [1.0, 2.0, 3])
    # d_gain["PL"] = fuzz.trimf(d_gain.universe, [2.0, 3, 3])

    d_rule1 = ctrl.Rule(f_error["NL"] & f_errorder["NL"], d_gain["PS"])
    d_rule2 = ctrl.Rule(f_error["NL"] & f_errorder["NM"], d_gain["NS"])
    d_rule3 = ctrl.Rule(f_error["NL"] & f_errorder["NS"], d_gain["NL"])
    d_rule4 = ctrl.Rule(f_error["NL"] & f_errorder["Z"], d_gain["Z"])
    d_rule5 = ctrl.Rule(f_error["NL"] & f_errorder["PS"], d_gain["NL"])
    d_rule6 = ctrl.Rule(f_error["NL"] & f_errorder["PM"], d_gain["NM"])
    d_rule7 = ctrl.Rule(f_error["NL"] & f_errorder["PL"], d_gain["PS"])

    d_rule8 = ctrl.Rule(f_error["NM"] & f_errorder["NL"], d_gain["PS"])
    d_rule9 = ctrl.Rule(f_error["NM"] & f_errorder["NM"], d_gain["NS"])
    d_rule10 = ctrl.Rule(f_error["NM"] & f_errorder["NS"], d_gain["NL"])
    d_rule11 = ctrl.Rule(f_error["NM"] & f_errorder["Z"], d_gain["Z"])
    d_rule12 = ctrl.Rule(f_error["NM"] & f_errorder["PS"], d_gain["NM"])
    d_rule13 = ctrl.Rule(f_error["NM"] & f_errorder["PM"], d_gain["NS"])
    d_rule14 = ctrl.Rule(f_error["NM"] & f_errorder["PL"], d_gain["Z"])

    d_rule15 = ctrl.Rule(f_error["NS"] & f_errorder["NL"], d_gain["Z"])
    d_rule16 = ctrl.Rule(f_error["NS"] & f_errorder["NM"], d_gain["NS"])
    d_rule17 = ctrl.Rule(f_error["NS"] & f_errorder["NS"], d_gain["NM"])
    d_rule18 = ctrl.Rule(f_error["NS"] & f_errorder["Z"], d_gain["Z"])
    d_rule19 = ctrl.Rule(f_error["NS"] & f_errorder["PS"], d_gain["NS"])
    d_rule20 = ctrl.Rule(f_error["NS"] & f_errorder["PM"], d_gain["NS"])
    d_rule21 = ctrl.Rule(f_error["NS"] & f_errorder["PL"], d_gain["Z"])

    d_rule22 = ctrl.Rule(f_error["Z"] & f_errorder["NL"], d_gain["Z"])
    d_rule23 = ctrl.Rule(f_error["Z"] & f_errorder["NM"], d_gain["NS"])
    d_rule24 = ctrl.Rule(f_error["Z"] & f_errorder["NS"], d_gain["NS"])
    d_rule25 = ctrl.Rule(f_error["Z"] & f_errorder["Z"], d_gain["Z"])
    d_rule26 = ctrl.Rule(f_error["Z"] & f_errorder["PS"], d_gain["NS"])
    d_rule27 = ctrl.Rule(f_error["Z"] & f_errorder["PM"], d_gain["NS"])
    d_rule28 = ctrl.Rule(f_error["Z"] & f_errorder["PL"], d_gain["Z"])

    d_rule29 = ctrl.Rule(f_error["PS"] & f_errorder["NL"], d_gain["Z"])
    d_rule30 = ctrl.Rule(f_error["PS"] & f_errorder["NM"], d_gain["Z"])
    d_rule31 = ctrl.Rule(f_error["PS"] & f_errorder["NS"], d_gain["Z"])
    d_rule32 = ctrl.Rule(f_error["PS"] & f_errorder["Z"], d_gain["Z"])
    d_rule33 = ctrl.Rule(f_error["PS"] & f_errorder["PS"], d_gain["Z"])
    d_rule34 = ctrl.Rule(f_error["PS"] & f_errorder["PM"], d_gain["Z"])
    d_rule35 = ctrl.Rule(f_error["PS"] & f_errorder["PL"], d_gain["Z"])

    d_rule36 = ctrl.Rule(f_error["PM"] & f_errorder["NL"], d_gain["PL"])
    d_rule37 = ctrl.Rule(f_error["PM"] & f_errorder["NM"], d_gain["NS"])
    d_rule38 = ctrl.Rule(f_error["PM"] & f_errorder["NS"], d_gain["PS"])
    d_rule39 = ctrl.Rule(f_error["PM"] & f_errorder["Z"], d_gain["Z"])
    d_rule40 = ctrl.Rule(f_error["PM"] & f_errorder["PS"], d_gain["PS"])
    d_rule41 = ctrl.Rule(f_error["PM"] & f_errorder["PM"], d_gain["PS"])
    d_rule42 = ctrl.Rule(f_error["PM"] & f_errorder["PL"], d_gain["PL"])

    d_rule43 = ctrl.Rule(f_error["PL"] & f_errorder["NL"], d_gain["PL"])
    d_rule44 = ctrl.Rule(f_error["PL"] & f_errorder["NM"], d_gain["NS"])
    d_rule45 = ctrl.Rule(f_error["PL"] & f_errorder["NS"], d_gain["PS"])
    d_rule46 = ctrl.Rule(f_error["PL"] & f_errorder["Z"], d_gain["Z"])
    d_rule47 = ctrl.Rule(f_error["PL"] & f_errorder["PS"], d_gain["PS"])
    d_rule48 = ctrl.Rule(f_error["PL"] & f_errorder["PM"], d_gain["PS"])
    d_rule49 = ctrl.Rule(f_error["PL"] & f_errorder["PL"], d_gain["PL"])
    
    d_rules = [d_rule1, d_rule2, d_rule3, d_rule4, d_rule5, d_rule6, d_rule7,
               d_rule8, d_rule9, d_rule10, d_rule11, d_rule12, d_rule13, d_rule14,
               d_rule15, d_rule16, d_rule17, d_rule18, d_rule19, d_rule20, d_rule21,
               d_rule22, d_rule23, d_rule24, d_rule25, d_rule26, d_rule27, d_rule28,
               d_rule29, d_rule30, d_rule31, d_rule32, d_rule33, d_rule34, d_rule35,
               d_rule36, d_rule37, d_rule38, d_rule39, d_rule40, d_rule41, d_rule42,
               d_rule43, d_rule44, d_rule45, d_rule46, d_rule47, d_rule48, d_rule49,]
    
    for rule in d_rules:
        d_controller.addrule(rule)

    # plot_functions = True

    # if plot_functions:
    #     f_error.view()
    #     f_errorder.view()
    #     p_gain.view()
    #     i_gain.view()
    #     d_gain.view()
    #     plt.show()

def fuzzy_pid(error, error_dif):

    global p_out
    global i_out
    global d_out
    global p_simulation
    global p_controller
    global i_simulation
    global i_controller
    global d_controller
    global d_simulation

    p_simulation.input["error"] = error
    p_simulation.input["errorder"] = error_dif
    p_simulation.compute()
    p_out = p_simulation.output["gain"]

    i_simulation.input["error"] = error
    i_simulation.input["errorder"] = error_dif
    i_simulation.compute()
    i_out = i_simulation.output["gain"]
    
    d_simulation.input["error"] = error
    d_simulation.input["errorder"] = error_dif
    d_simulation.compute()
    d_out = d_simulation.output["gain"]          

def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error_dif
    global error
    global error_int
    global error_prev
    global max_error_der
    global max_neg_error_der
    # global max
    global max_neg
    global p_out
    global i_out
    global d_out
    global kP
    global kD
    global kI
    global err_history
    global t_prev
    global image_input
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown
    global error_log
    global time_log
    global control_log

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape

    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  
    # also get the side in which the track mark "is"
    
    message = Twist()
    
    if line:
    # if there even is a line in the image:
    # (as the camera could not be reading any lines)      
        x = line['x']

        # error:= The difference between the center of the image
        # and the center of the line
        error = (width//2 - x + 10)/175
        tstamp = time.time()

        message.linear.x = LINEAR_SPEED
        just_seen_line = True
        # print('just seen the line')

        # plot the line centroid on the image
        cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)

    else:
        # There is no line in the image. 
        # Turn on the spot to find it again. 
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        message.linear.x = 0.0

    if mark_side != None:
        print("mark_side: {}".format(mark_side))

        if (mark_side == "right") and (finalization_countdown == None) and \
            (abs(error) <= MAX_ERROR) and (not just_seen_right_mark):

            right_mark_count += 1

            if right_mark_count > 1:
                # Start final countdown to stop the robot
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                print("Finalization Process has begun!")

            
            just_seen_right_mark = True
    else:
        just_seen_right_mark = False

    
    # Determine the speed to turn and get the line in the center of the camera.
    if just_seen_line and should_move:
        dt = tstamp - t_prev
        if dt > 0.0:
            err_history.put(error)
            error_int += error
            error_log.append(error)
            time_log.append(tstamp)
            if err_history.full():
                error_int -= err_history.get()
            error_dif = error - error_prev
            # if error >= max:
            #     max = error
            # if error <= max_neg:
            #     max_neg = error
            # if error_dif >= max_error_der:
            #     max_error_der = error_dif
            # if error_dif <= max_neg_error_der:
            #     max_neg_error_der = error_dif
            # print('max error, max error neg', max, max_neg)
            # print('max error_der, max error_neg_der:', max_error_der, max_neg_error_der) 
            fuzzy_pid(error, error_dif)
            # print('p_out, i_out, d_out', p_out, i_out, d_out)
            kP = p_out
            kI = i_out
            kD = d_out
            print('kP, kI, kD', kP, kI, kD)
            u = (kP * error) + (kI * error_int * dt) + (kD * error_dif / dt)
            error_prev = error
            t_prev = tstamp
            message.angular.z = u
            print(u)
            control_log.append(u)
    # print('angular:', message.angular.z)

    



    # Plot the boundaries where the image was cropped
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)

    # Uncomment to show the binary picture
    # cv2.imshow("mask", mask)

    # Show the output image to the user
    cv2.imshow("output", output)
    

    # Print the image for 5milis, then resume execution
    cv2.waitKey(5)

    # Check for final countdown
    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1

        elif finalization_countdown == 0:
            should_move = False

    if not should_move and error_log and time_log:
        f1 = plt.figure(1)
        plt.plot(time_log, error_log)
        plt.xlabel("Time")
        plt.xlim(min(time_log), max(time_log))
        plt.ylabel("Error")
        plt.yticks(np.arange(math.floor(min(error_log)), math.ceil(max(error_log)) + 1, 0.5))
        plt.title("Fuzzy PID Controller Error over Time")

    if not should_move and control_log and time_log:
        f2 = plt.figure(2)
        plt.plot(time_log, control_log)
        plt.xlabel("Time")
        plt.xlim(min(time_log), max(time_log))
        plt.ylabel("Control Signal - Angular Velocity")
        plt.title("Control Signal Over Time")

    if not should_move and time_log and error_log:
        plt.show()
        rclpy.shutdown()

    # Publish the message to 'cmd_vel'
    if should_move:
        publisher.publish(message)
    else:
        empty_message = Twist()
        publisher.publish(empty_message)

class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception)


def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


# Go to AUV mode
go_auv = TargetMode.Goal()
go_auv.target_mode = TargetMode.Goal.ORCA_MODE_AUV

# Go to ROV mode
go_rov = TargetMode.Goal()
go_rov.target_mode = TargetMode.Goal.ORCA_MODE_ROV

# Go home (1m deep)
go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))

# Dive to 8m
dive = FollowWaypoints.Goal()
dive.poses.append(make_pose(x=0.0, y=0.0, z=-8.0))

# Big loop, will eventually result in a loop closure
delay_loop = FollowWaypoints.Goal()
delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-3.0))
# for _ in range(2):
#     delay_loop.poses.append(make_pose(x=20.0, y=-13.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=10.0, y=-23.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=-10.0, y=-8.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))


# Send a goal to an action server and wait for the result.
# Cancel the goal if the user hits ^C (KeyboardInterrupt).
def send_goal(node, action_client, send_goal_msg) -> SendGoalResult:
    goal_handle = None

    try:
        action_client.wait_for_server()

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg)
        rclpy.spin_until_future_complete(node, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError('Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

        print('Goal completed')
        return SendGoalResult.SUCCESS

    except KeyboardInterrupt:
        # Cancel the goal if it's still active
        # TODO(clyde): this seems to work, but a second exception is generated -- why?
        if (goal_handle is not None and
                (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                 GoalStatus.STATUS_EXECUTING == goal_handle.status)):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                raise RuntimeError('Exception while canceling goal: {!r}'.format(cancel_future.exception()))

            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            if len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            if cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')

            print('Goal canceled')
            return SendGoalResult.CANCELED


def main():
    global node
    node = None
    set_target_mode = None
    follow_waypoints = None

    rclpy.init()
    global publisher

    try:
        node = rclpy.create_node("mission_runner")

        set_target_mode = ActionClient(node, TargetMode, '/set_target_mode')
        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')

        publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
        subscription = node.create_subscription(Image, '/stereo_left',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)

        print('>>> Setting mode to AUV <<<')
        if send_goal(node, set_target_mode, go_auv) == SendGoalResult.SUCCESS:
            print('>>> Executing mission <<<')
            send_goal(node, follow_waypoints, delay_loop)
            timer = node.create_timer(TIMER_PERIOD, timer_callback)
            start_time = node.get_clock().now()

            start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
            stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)
            fuzzy_membership()
            # print("testing sa main")
            rclpy.spin(node)

            # print('>>> Setting mode to ROV <<<')
            # send_goal(node, set_target_mode, go_rov)

            

            print('>>> Mission complete <<<')
        else:
            print('>>> Failed to set mode to AUV, quit <<<')

    finally:
        if set_target_mode is not None:
            set_target_mode.destroy()
        if follow_waypoints is not None:
            follow_waypoints.destroy()
        if node is not None:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
