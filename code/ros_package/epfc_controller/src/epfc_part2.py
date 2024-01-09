#!/usr/bin/env python3

from tf.transformations import euler_from_quaternion as eq
import matplotlib.pyplot as graph
import rospy as rp 
import numpy as calc
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty as place_holder
from geometry_msgs.msg import Twist as tw


##############################################################################################################
# Class of our epfc controller object
##############################################################################################################

class ExtendedPotentialFieldController:
    def __init__(self):

        # Initilaise drone arrays
        self.drone_pos = calc.array([0.0, 0.0])
        self.drone_vel = calc.array([0.0, 0.0])

        # Define the controller parameters
        self.lamda_one = 1.0
        self.lamda_two = 0.28
        self.etha_one = 0.029

        # Initialize graphing arrays
        self.distance_bw_drone_target = calc.array([])
        self.distance_bw_drone_obstacle = calc.array([])
        self.tracking_error = calc.array([])
        self.timestamp = calc.array([])


        # ########### Experiment 3 ###############
        # #Define waypoints and obstacles
        # self.target_pos= calc.matrix((calc.array((4.2,0.0)))).T
        # self.obstacle_pos= calc.matrix(calc.empty).T


        # ############ Experiment 4 ###############
        # # Define waypoints and obstacles
        # self.target_pos= calc.matrix((calc.array([5.25,0.0]))).T
        # self.obstacle_pos= calc.matrix((calc.array([2.5,0.0]))).T


        ########### Experiment 6 ###############
        #Define waypoints and obstacles
        self.target_pos= calc.matrix((calc.array((2.7,1.0))  )).T
        self.obstacle_pos= calc.matrix(calc.empty).T


        self.target_count = calc.shape(self.target_pos)[1]  
        self.obstacle_count = calc.shape(self.obstacle_pos)[1]  

        if (calc.shape(self.obstacle_pos)[0] == 1) and (calc.shape(self.obstacle_pos)[1] == 1):
            self.obstacle_count = 0

        rp.init_node('epfc_controller')

        self.s_drone_pose = rp.Subscriber("/drone/gt_pose", Pose, self.cb_drone_pose)
        self.p_start = rp.Publisher('/drone/takeoff', place_holder, queue_size=7)
        self.p_epfc = rp.Publisher('/cmd_vel', tw, queue_size=10)
        self.s_drone_vel = rp.Subscriber("/drone/gt_vel", tw, self.cb_drone_vel)

        while(self.p_start.get_num_connections() < 1):
            pass

        self.p_start.publish(place_holder())
        self.init_time = rp.get_time()

        while(self.p_epfc.get_num_connections() < 1 or self.s_drone_vel.get_num_connections() < 1 or self.s_drone_pose.get_num_connections() < 1):
            pass



    # Callback fn for Pose message of drone pose
    def cb_drone_pose(self, msg):

        self.drone_pos = calc.array([msg.position.x, msg.position.y]) 
        (r, p, self.yw) = eq ([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]) 


    # Callback fn for Twist message of drone velocity
    def cb_drone_vel(self, msg):
        self.drone_vel = calc.array([msg.linear.x, msg.linear.y]) 



    # Controller fn
    def compute(self):

        p_vel_msg = tw()

        waypoint_pointer = 0 # waypoint number 
        obs_range = 1.0 # range of obstacle avoidance (1m)
        
        publish_frequency = rp.Rate(48)

        while ((not rp.is_shutdown())):
            
            rp.loginfo("Going to WAYPOINT "+str(waypoint_pointer+1)+"...")


            # We consider target to be reached by drone if it is less than 0.1m in proximity           
            #while(calc.linalg.norm(self.drone_pos - (self.target_pos[:,waypoint_pointer]).T) >= 0.099): 
            
            while(rp.get_time() - self.init_time <10.0): 

                # velocity due to attractive potential
                pfc_attr_vel = -self.lamda_one*(self.drone_pos - self.target_pos[:,waypoint_pointer].T) 
                
                # velocity due to repulsive potential
                pfc_rep_vel = calc.array([0,0]) 

                for obs_index in range(0,self.obstacle_count):
                    if (calc.linalg.norm(self.drone_pos - (self.obstacle_pos[:,obs_index]).T) <= obs_range):
                        term_one = (calc.linalg.norm((self.drone_pos - (self.obstacle_pos[:,obs_index]).T)))**4
                        term_two = (self.obstacle_pos[:,obs_index])
                        pfc_rep_vel = (self.etha_one/term_one) * (self.drone_pos - term_two.T)
                    else:
                        pfc_rep_vel = pfc_rep_vel + calc.array([0,0])

                pfc_vel = pfc_attr_vel + pfc_rep_vel
                epfc_vel_i_frame = pfc_vel - self.lamda_two*(self.drone_vel)

                # Transform pose to body frame from initial frame
                T_matrix = calc.matrix([[calc.cos(self.yw), calc.sin(self.yw)], [-calc.sin(self.yw), calc.cos(self.yw)]]) 

                epfc_vel_b_frame = (calc.matmul(T_matrix, epfc_vel_i_frame.T)).T 

                # Clipping value to (-1,1) to be in accordance to drone's inout requirement
                epfc_vel_b_frame = calc.clip(epfc_vel_b_frame, -1, 1) 

                
                # message publish to cmd_vel to move drone

                p_vel_msg.linear.x = epfc_vel_b_frame[0,0]
                p_vel_msg.linear.y = epfc_vel_b_frame[0,1]

                self.p_epfc.publish(p_vel_msg)

                current_time = rp.get_time()

                #record data in array   
                #self.distance_bw_drone_target= calc.append(self.distance_bw_drone_target, (self.target_pos[0,waypoint_pointer]).T - self.drone_pos[0])                
                #self.distance_bw_drone_obstacle= calc.append(self.distance_bw_drone_obstacle, calc.linalg.norm((self.obstacle_pos[0,0]).T - self.drone_pos[0]))
                self.tracking_error = calc.append(self.tracking_error, self.drone_pos[0] - (self.target_pos[0,waypoint_pointer]).T)
                self.timestamp = calc.append(self.timestamp, current_time);

                publish_frequency.sleep()

            self.p_epfc.publish(tw())

            # increment waypoint index once a waypoint is reached
            waypoint_pointer = waypoint_pointer + 1

            # Once all waypoints are reached, display the graph
            if(waypoint_pointer >= self.target_count):

                graph.title('ePFC controller') 
                graph.xlabel(' time (s) ')
                #graph.ylabel(' relative distance (m) ') 
                graph.ylabel(' error (m) ')  
                #graph.plot(self.timestamp, self.distance_bw_drone_target, color='blue', label="target")
                #graph.plot(self.timestamp, self.distance_bw_drone_obstacle, color='green',label="obstacle")
                graph.plot(self.timestamp, self.tracking_error, color='green',label="tracking error")
                graph.legend()
                graph.show()

                break


if __name__ == '__main__':
    try:
        ePFC = ExtendedPotentialFieldController()
        ePFC.compute()
    except rp.ROSInterruptException:
        pass
