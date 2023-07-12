#!/usr/bin/env python3


'''
Gkoutzas Nikolaos           4343
Katsilieris Athanasios      3247
Naka Amalia                 3295
'''


from math import atan2 , sqrt 
import rospy , time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 


rospy.init_node('topic_publisher')

AM = 4343                                                               # (AM)

# initial position
position_init_X = 0                                                     # (meters)
position_init_Y = 0                                                     # (meters)
orientation_init = 0                                                    # (rad)

# intermediate position
position_interm_X = round(AM / 200)                                     # (meters)
position_interm_Y = round(AM / 400)                                     # (meters)
orientation_interm = atan2(position_interm_Y , position_interm_X)       # (rad)

orientation_towards_final = -1.57079633                                 # (rad) -> -90deg

position_final_X = round(AM / 200)                                      # (meters)
position_final_Y = -round(AM / 400)                                     # (meters)
orientation_final = ((AM / 2) / 1000)                                   # (rad)

total_time = 0                                                          # (seconds)
total_distance = 0                                                      # (meters)

bool_ = True                                                            # useful variable in order to sleep for 1 sec before start the first step of simulation

rate = rospy.Rate(10)
move = Twist()
publish_ = rospy.Publisher('/pioneer/RosAria/cmd_vel' , Twist , queue_size=1)


'''
>> compute anguar velocity of robot
1st parameter: current time (starts from 0 sec)
2nd parameter: total time of rotation
3rd parameter: final orientation 
4th parameter: initial orientation
'''
def angular_velocity(t , t_interm , final_orient , first_orient):
    a1 = 0
    a2 = (3 / pow(t_interm , 2) ) * (final_orient - first_orient)
    a3 = (-2 / pow(t_interm , 3) ) * (final_orient - first_orient)
    return a1 + 2*a2*t + 3*a3*pow(t , 2)


'''
>> compute angle of robot
1st parameter: current time (starts from 0 sec)
2nd parameter: total time of rotation
3rd parameter: final orientation 
4th parameter: initial orientation
'''
def angle(t , t_interm , final_orient , first_orient):
    global orientation_init
    
    a0 = orientation_init
    a1 = 0
    a2 = (3 / pow(t_interm , 2) ) * (final_orient - first_orient)
    a3 = (-2 / pow(t_interm , 3) ) * (final_orient - first_orient)
    return a0 + a1*t + a2*pow(t , 2) + a3*pow(t , 3)


'''
>> compute linear velocity of robot
1st parameter: current time (starts from 0 sec)
2nd parameter: total time of rotation
3rd parameter: final position in X axis 
4th parameter: final position in Y axis
5th parameter: initial position in X axis 
6th parameter: initial position in Y axis
'''
def linear_velocity(t , t_interm , final_position_X , final_position_Y , first_position_X , first_position_Y):
    a1 = 0
    a2 = (3 / pow(t_interm , 2) ) * sqrt( pow(final_position_X - first_position_X , 2) + pow(final_position_Y - first_position_Y , 2) )
    a3 = (-2 / pow(t_interm , 3) ) * sqrt( pow(final_position_X - first_position_X , 2) + pow(final_position_Y - first_position_Y , 2) )
    return a1 + 2*a2*t + 3*a3*pow(t , 2)


'''
>> compute elapsed distance of robot
1st parameter: current time (starts from 0 sec)
2nd parameter: total time of rotation
3rd parameter: final position in X axis 
4th parameter: final position in Y axis
5th parameter: initial position in X axis 
6th parameter: initial position in Y axis
'''
def elapsed_distance(t , t_interm , final_position_X , final_position_Y , first_position_X , first_position_Y):
    global position_init_X

    a0 = position_init_X
    a1 = 0
    a2 = (3 / pow(t_interm , 2) ) * sqrt( pow(final_position_X - first_position_X , 2) + pow(final_position_Y - first_position_Y , 2) )
    a3 = (-2 / pow(t_interm , 3) ) * sqrt( pow(final_position_X - first_position_X , 2) + pow(final_position_Y - first_position_Y , 2) )
    return a0 + a1*t + a2*pow(t , 2) + a3*pow(t , 3)
    



# option = 1 -> print messages for rotation , option = 2 -> print messages for movement
def print_messages(option , time__ , final_X , first_X , final_Y , first_Y , current_time , start_time):    
    global total_time , total_distance    

    if(option):
        print("Angle: " + "{:.7f}".format(angle(current_time - start_time , time__ , final_X , first_X)) , end = " rad  ,  " , flush = True)
        print("Angular velocity: " + "{:.7f}".format(angular_velocity(current_time - start_time , time__ , final_X , first_X)) + \
              " rad/sec.  ,  " + "Elasped time: " + "{:.7f}".format(current_time - start_time) + " sec.  ,  " + "Total time: " \
              + "{:.7f}".format(total_time + current_time - start_time) + " sec.")
    else:
        print("Linear velocity: " + "{:.7f}".format(linear_velocity(current_time - start_time , time__ , final_X , first_X , final_Y , first_Y)) + \
              " m/sec. ,  " + "Elapsed distance: " + "{:.7f}".format(elapsed_distance(current_time - start_time , time__ , final_X , final_Y , first_X , first_Y))\
              + " met.  ,  Total distance: " + "{:.7f}".format(total_distance + \
              elapsed_distance(current_time - start_time , time__ , final_X , final_Y , first_X , first_Y)) + " met.  ,  " + "Elasped time: " \
              + "{:.7f}".format(current_time - start_time) + \
              " sec.  ,  " + "Total time: " + "{:.7f}".format(total_time + current_time - start_time) + " sec.")



'''
function that computes every step (rotation/movement)
'''
def action(option , time_ , pos_first_X , pos_final_X , pos_first_Y , pos_final_Y , orient_first , orient_final):
    global total_time , total_distance , bool_  , list_timer
    start_time = rospy.get_time()       # get the simulation time from ros
    current_time = rospy.get_time()     # get the simulation time from ros
    

    while( not rospy.is_shutdown() ):   # while user does not press [Ctrl+C] button ,loop...
        if(bool_):                      # this if statement will execute only once.If that statement did not exist,somethimes the simulation
                                        # would not execute the first step ,which is the first rotation (for some reason) ,so
                                        # we thought to delay the simulation at the beginning for 1 second and this method does work excellent (every time). 
            time.sleep(.5)              # sleep for 0.5 seconds
            bool_ = False               # change bool_ variable to False and do not make it True again
            start_time = rospy.get_time()   # get the simulation time from ros (again)
            current_time = rospy.get_time() # get the simulation time from ros (again)
                

        if(current_time - start_time >= time_):     # if time passed,then
            break                                   # break the loop and continue to the next tep (rotation or movement)

        else:                                       # otherwise (if time did not passed yet)
            if(option):     # for rotation
                print_messages(True , time_ , orient_final , orient_first , None , None , current_time , start_time)    # print some messages
                move.angular.z = angular_velocity(current_time - start_time , time_ , orient_final , orient_first)      # rotate the robot to the current angle
                                                                                                                        # with a limited angular speed
            else:           # for movement
                print_messages(False , time_ , pos_final_X , pos_final_Y , pos_first_X , pos_first_Y , current_time , start_time)           # print some messages
                move.linear.x = linear_velocity(current_time - start_time , time_ , pos_final_X , pos_final_Y , pos_first_X  , pos_first_Y) # move the robot
                                                                                                                                            # forward with a 
                                                                                                                                            # limited linear speed 
            publish_.publish(move)                                                                                                          # publish move variable
                                                                                                                                            # to cmd_vel topic
            current_time = rospy.get_time()     # update current time (get the info from ros)

    if( rospy.is_shutdown() ):      # check if user pressed [Ctrl+C]
        print("\n\nSimulation interrupted by user...")    # simulation interrupted
        print("Statistics were not calculated due to simulation interruption.\n")
        exit()                                              # exit the simulation
    
    total_time += current_time - start_time     # update total time of simulation

    if(not option):     # for movement (only) , update total distance, because in rotation, distance does not change
        total_distance += elapsed_distance(current_time - start_time , time_ , pos_final_X , pos_final_Y , pos_first_X , pos_first_Y)

#-------------------------------------------------------------------------------------------------------------------------------------------------------

# call action() function with specific parameters
# the times were counted so that we have limited angular/linear speed

action(True , 1.2 , None , None , None , None , orientation_init , orientation_interm)

action(False , 190 , position_init_X , position_interm_X , position_init_Y , position_interm_Y , None , None)       # move for 190 seconds

action(True , 5 , None , None , None , None , orientation_interm , orientation_towards_final)                       # rotate for 5 seconds

action(False , 210 , position_interm_X , position_final_X , position_interm_Y , position_final_Y , None , None)     # move for 210 seconds

action(True , 10 , None , None , None , None , orientation_towards_final , orientation_final)                        # rotate for 10 seconds

print("\n\nTotal time:     " + "{:.7f}".format(total_time) + " seconds")
print("Total distance: " + "{:.7f}".format(total_distance) + " meters\n")