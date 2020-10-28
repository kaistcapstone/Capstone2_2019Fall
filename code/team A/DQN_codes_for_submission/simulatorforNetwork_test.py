#!/usr/bin/env python3
import cv2
import numpy as np
# import rospy, roslib, rospkg
import yaml, sys, time, random

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Vector3
# from tt_core_msgs.msg import Vector3DArray, ImagePoint
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage

#########################################
"Very Important"
# The parameters below are actually used in DQN
# observation_space
# action_space
# reset -> returns self.frame_gray -> size affected by simulator, debug_scale_gray
# step(action)
# get_episode_rewards

# Changing total # of actions -> should modify dqn_model.py too
##########################################

#############################################################################
# For both bigger and smaller map in fame_debug after execution, 10 pixels = one box
#map size multiplier - added by our team
import math
mm = 1

#simulator = {"width":31 , "height":31, "center":15, "resol":1}
simulator = {"width":31*mm -1 *(mm-1) , "height":31*mm -1*(mm-1), "center":15*mm, "resol":1}
map_param = {"width":101 , "height":101 , "center":50 , "resol":1, "scale":5}
# walls_samples = [[1.2,2.0],[1.4,2.0],[1.6,2.0],[1.2,4.0],[1.4,4.0],[1.6,4.0],[1.2,-2.0],[1.4,-2.0],[1.6,-2.0],[1.2,-4.0],[1.4,-4.0],[1.6,-4.0]]

#walls_samples = [[1.5 * mm,-30.0 * mm],[30.4 * mm, 0.7 * mm],[-30.4 * mm,-0.7 * mm]]
###########################################################################

############################################################################
# The RGB-d Camera we're using has FOV of around 80 degrees. For safety margin, set this at 60.
# Refer to "intel.com/content/www/us/en/support/articles/000030385/emerging-technologies/intel-realsense-technology.html"
# Camera field of view in degrees
camera_fov = 55  #120
###############################################################################
ball_blind_ratio = 1/np.tan(camera_fov/2*np.pi/180)
ball_blind_bias = 2 #5

# x is height and y is width
high_reward_region_x = [0]
high_reward_region_y = [0,1]
low_reward_region_x = [1,2]
low_reward_region_y = [0,1]
penalty_region1_x = [0,2]
penalty_region1_y = [1,3]
penalty_region2_x = [0,2]
penalty_region2_y = [-4,-2]
penalty_region3_x = [0,6]
penalty_region3_y = [-3,3]
# simulator["height"] - 4   sets the center of rotation at base(x=0)of the simulator frame.
center_of_rotation_x = simulator["height"] - 9 #simulator["height"] - 4
center_of_rotation_y = 0 #simulator["center"]

# Change center_of_rotation_x value to alter the center of rotation.
axisdx = center_of_rotation_x - simulator["height"] + 4
lidardx = axisdx
lidardy = 0 #5



trans_scale = int(simulator["resol"]/map_param["resol"])

###############################################################################
# Set how many degrees to rotate each step
rot_scale = 9
################################################################################

debug_scale = 10
debug_scale_gray = 3

max_iter = 500 #220 #180 #300 #130 #99


class Task:
    def __init__(self, debug_flag=False, test_flag=False, state_blink=True, state_inaccurate=True):
        self.frame = np.zeros((simulator["height"],simulator["width"],1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray,simulator["width"]*debug_scale_gray,1), np.uint8)
# self.balls are going to be all the balls randomly generated which are inside the walls
        self.balls = []
        self.balls_prev = []
###################################################################################################
# The balls that are picked up(that went into reward region)
# The balls that are 'visible' to the robot(within fov of robot)
        self.global_trace = np.zeros((map_param["height"],map_param["width"],1),np.uint8)
        self.balls_picked = []
        self.balls_inscreen = []
        self.ball_inscreen_history = 0
        self.wall_approaching_history = 0
        self.cont_rot_history = 0
        self.move_back_history = 0
        self.balls_in_contact = False
##################################################################################################
        self.obstacles = []
        self.episode_rewards = []
        self.score = 0
        self.iter = 0
        self.done = False

        self.write_flag = False
        self.debug_flag = debug_flag
        self.test_flag = test_flag
        self.ball_inscreen_flag = 0
###################################################################################################
        self.trace =[[simulator["center"],simulator["height"]]]
###################################################################################################
        self.state_blink = state_blink
        self.state_inaccurate = state_inaccurate

        ## DQN parameters

        self.observation_space = self.frame_gray.copy()
        self.action_space = np.array(range(4))

        # rospack = rospkg.RosPack()
        # root = rospack.get_path('tt_rl_motion_planner')
        # path = root+"/config/map_gen.yaml"
        path = "./map_gen.yaml"
        stream = open(path, 'r')
        self._params = yaml.load(stream)

        # self.reset(max_balls=20, max_walls=3)
        return

    def reset(self, max_balls=6, max_walls=5):
        self.frame = np.zeros((simulator["height"],simulator["width"],1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray,simulator["width"]*debug_scale_gray,1), np.uint8)
        self.balls = []
        self.balls_prev = []
        self.obstacles = []
        self.score = 0
        self.iter = 0
        self.done = False
        self.write_flag = False
        self.ball_inscreen_flag = 0
###################################################################################################
        self.ball_inscreen_history = 0
        self.wall_approaching_history = 0
        self.cont_rot_history = 0
        self.move_back_history = 0
        self.balls_picked = []
        self.balls_in_contact = False
        self.global_trace = np.zeros((map_param["height"],map_param["width"],1),np.uint8)
        self.trace =[[simulator["center"],simulator["height"]]]
###################################################################################################

        if len(self.episode_rewards)%5000 == 0 and not self.test_flag:
            self.write_flag = True
            out_directory = "data/video/tt.video."+format(len(self.episode_rewards)/5000,"06")+".mp4"

        if self.test_flag:
            self.write_flag = Truecos
            out_directory = "data/video_test/tt.video."+format(len(self.episode_rewards),"06")+".mp4"

        if self.write_flag:
            codec = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 10
            self.video = cv2.VideoWriter(out_directory, codec, fps, (simulator["width"]*debug_scale,simulator["height"]*debug_scale))

        #num_walls = int((max_walls+1)*random.random())
        #walls_sampled = random.sample(walls_samples, num_walls)
####################################################################################################
        #rand_direction = random.random()
	#if rand_direction >= 0.666:
        #    walls_sampled = [[random.random()+0.7,-10*random.random()-20],[-random.random()-0.2,-10*random.random()-20],\
        #                [10*random.random()+20,0.5*random.random()+0.4],[-10*random.random()-20,-0.5*random.random()-0.4]]
        #elif rand_direction >= 0.333:
        #    walls_sampled = [[random.random()+0.7,10*random.random()+20],[-random.random()-0.2,10*random.random()+20],\
        #                [-10*random.random()-20,0.5*random.random()+0.4],[10*random.random()+20,-0.5*random.random()-0.4]]
        #else:
        #    walls_sampled = []
        #if rand_direction >= 0.5:
        #    walls_sampled = [[random.random()+0.7,-10*random.random()-20],[-random.random()-0.2,-10*random.random()-20],\
        #                [10*random.random()+20,0.5*random.random()+0.4],[-10*random.random()-20,-0.5*random.random()-0.4]]
        #else:
        #    walls_sampled = [[random.random()+0.7,10*random.random()+20],[-random.random()-0.2,10*random.random()+20],\
        #                [-10*random.random()-20,0.5*random.random()+0.4],[10*random.random()+20,-0.5*random.random()-0.4]]
#################################################################################################
# Changed so that walls always generate as similar as our demonstration environment. One pixel was assumed as the actual
# size of the ball, which is around 7.5 cm. Also, the starting point is randomly distributed in one of the vicinities of
# the exit of the 'maze'.
# The initial random angle is not utilized here. See the part after self.balls was appended.
        rnum = random.choice([-1,1])
        rnum_dx = (0.04 * random.random()) * random.choice([-1,1])
        rnum_dy = (0.09 * random.random() + 0.03) * -rnum
#####################################################################################################################
        walls_sampled = [[-0.86 + rnum_dx,-15.0],[0.6 + rnum_dx,15.0],\
                        [-15.0,-0.1*rnum + rnum_dy],[15.0,0.7*rnum + rnum_dy]]

        obstacles_temp = []
        for wall in walls_sampled:
            if abs(wall[1]) >= abs(wall[0]) and abs(wall[0]) <= 0.4:
                point_start = -1*map_param["center"] + int(60 + 15.5*(rnum-1)) + 51 * rnum_dy
                point_end = 1*map_param["center"] - 28 - 14*(rnum-1) + 36 * rnum_dy
                unit = (point_end-point_start)/210
                number = int(180 + 72*(rnum-1))

                for i in range(number):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])
            elif abs(wall[1]) >= abs(wall[0]) and abs(wall[0]) > 0.2:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"]
                unit = (point_end-point_start)/210

                for i in range(210):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])
            else:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"] + 5
                unit = (point_end-point_start)/210

                for i in range(210):
                    cx = (point_start + unit*i)
                    cy = (wall[1]*(map_param["center"]-(cx/wall[0])))
                    obstacles_temp.append([cx,cy])

        for obstacle in obstacles_temp:
            cx = obstacle[0]
            cy = obstacle[1]
            insert = True
            for wall in walls_sampled:
                if cx/wall[0] + cy/wall[1] > map_param["center"]:
                    insert = False
            if insert:
                self.obstacles.append([cx,cy])
#####################################################################################################################

# Changed so that 'max_balls' number of balls always randomly generate within the obstacle walls.
        while len(self.balls) < max_balls:
            #cx = int(1.0*random.random()*(map_param["height"]-2*trans_scale)+2*trans_scale)
            #cy = int(1.0*random.random()*map_param["width"]) - map_param["center"]
            #cx = #5*3
            #cy = #5*3
            cx = int(1.0*random.random()*(map_param["height"]-2*trans_scale)+2*trans_scale)
            cy = int(1.0*random.random()*map_param["width"]) - map_param["center"]
            insert = True
            for wall in obstacles_temp:
                # Make the balls certain distance away from the wall
                if (cx - wall[0]) ** 2 + (cy - wall[1]) ** 2 < 6 ** 2:
                    insert = False
                # Let the balls form centered around the map center and inside the area surrounded by the walls.
                if (cx - map_param["center"]) ** 2 + (cy - map_param["center"]) ** 2 < 5 ** 2:
                    insert = False
                for ws in walls_sampled:
                    if cx/ws[0] + cy/ws[1] >= map_param["center"]:
                        insert = False
            for ball in self.balls:
                # Prevent the balls from generating too densely distributed.
                if (cx - ball[0]) ** 2 + (cy - ball[1]) ** 2 < 6 ** 2:
                    insert = False
                #if 0 <= ball[0] <= penalty_region2_x[-1] + 2 and abs(ball[1]) <= penalty_region1_y[-1] + 1:
                #    insert = False
            # Prevent the balls from generating too close to the robot body.
            if cx ** 2 + cy **2 < 7 ** 2:
                insert = False
            if insert:
                self.balls.append([cx,cy])

####################################################################################################


        walls_sampled = [[-0.2 + rnum_dx,-15.0],[0.6 + rnum_dx,15.0],\
                        [-15.0,-0.1*rnum + rnum_dy],[15.0,0.7*rnum + rnum_dy]]
        obstacles_temp = []
        for wall in walls_sampled:
            if abs(wall[1]) >= abs(wall[0]) and abs(wall[0]) <= 0.4:
                point_start = -1*map_param["center"] + int(60 + 15.5*(rnum-1)) + 51 * rnum_dy
                point_end = 1*map_param["center"] - 28 - 14*(rnum-1) + 36 * rnum_dy
                unit = (point_end-point_start)/210
                number = int(180 + 72*(rnum-1))

                for i in range(number):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])
            elif abs(wall[1]) >= abs(wall[0]) and abs(wall[0]) > 0.2:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"]
                unit = (point_end-point_start)/210

                for i in range(210):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])
            else:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"] + 5
                unit = (point_end-point_start)/210

                for i in range(210):
                    cx = (point_start + unit*i)
                    cy = (wall[1]*(map_param["center"]-(cx/wall[0])))
                    obstacles_temp.append([cx,cy])

        obstacles_temptemp = []
        #print(len(obstacles_temptemp))
        for obstacle in obstacles_temp:
            cx = obstacle[0]
            cy = obstacle[1]
            insert = True
            for wall in walls_sampled:
                if cx/wall[0] + cy/wall[1] > map_param["center"]:
                    insert = False
            if insert:
                obstacles_temptemp.append([cx,cy])
        #print(len(obstacles_temptemp))
        number = int(180 + 72*(rnum-1))
        for i in range(number):
            self.obstacles.append(obstacles_temptemp[i])
#####################################################################################################################
        walls_sampled = [[-0.48 + rnum_dx,-15.0],[0.6 + rnum_dx,15.0],\
                        [-15.0,-0.1*rnum + rnum_dy],[15.0,0.7*rnum + rnum_dy]]
        obstacles_temp = []
        for wall in walls_sampled:
            if abs(wall[1]) >= abs(wall[0]) and 0.44 <= abs(wall[0]) <= 0.52:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"]
                unit = (point_end-point_start)/210

                for i in range(210):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])

            elif abs(wall[1]) >= abs(wall[0]) and abs(wall[0]) > 0.2:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"]
                unit = (point_end-point_start)/210

                for i in range(210):
                    cy = (point_start + unit*i)
                    cx = (wall[0]*(map_param["center"]-(cy/wall[1])))
                    obstacles_temp.append([cx,cy])
            else:
                point_start = -1.0*map_param["center"]
                point_end = 1.0*map_param["center"] + 5
                unit = (point_end-point_start)/210

                for i in range(210):
                    cx = (point_start + unit*i)
                    cy = (wall[1]*(map_param["center"]-(cx/wall[0])))
                    obstacles_temp.append([cx,cy])

        obstacles_temptemp = []
        #print(len(obstacles_temptemp))
        for obstacle in obstacles_temp:
            cx = obstacle[0]
            cy = obstacle[1]
            insert = True
            for wall in walls_sampled:
                if cx/wall[0] + cy/wall[1] > map_param["center"]:
                    insert = False
            if insert:
                obstacles_temptemp.append([cx,cy])
        #print(len(obstacles_temptemp))
        for i in range(26):
            self.obstacles.append(obstacles_temptemp[i])
        for i in range(30):
            k = i + 55
            self.obstacles.append(obstacles_temptemp[k])


#####################################################################################################################
        walls_sampled = [[-0.2 + rnum_dx,-15.0],[0.6 + rnum_dx,15.0],\
                        [-15.0,-0.1*rnum + rnum_dy],[15.0,0.7*rnum + rnum_dy]]

        #obstacles_temprnd = obstacles_temp[:]
        #random.shuffle(obstacles_temprnd)
        #obstacles_temprnd = obstacles_temprnd[0:int(len(obstacles_temp)/10)]
###############################################################################################

# Out of all 'max_balls'(it is a number) balls generated, put the x and y coordinates of the ball relative to the robot for the balls that are within the generated walls.
        #for i in range(max_balls):
        #    #cx = int(1.0*random.random()*(map_param["height"]-2*trans_scale)+2*trans_scale)
        #    #cy = int(1.0*random.random()*map_param["width"]) - map_param["center"]
        #    #cx = #5*3
        #    #cy = #5*3
        #    cx = int(1.0*random.random()*(map_param["height"]-2*trans_scale)+2*trans_scale)
        #    cy = int(1.0*random.random()*map_param["width"]) - map_param["center"]
        #    insert = True
        #    for wall in walls_sampled:
        #        if cx/wall[0] + cy/wall[1] >= map_param["center"]:
        #            insert = False
        #    if insert:
        #        self.balls.append([cx,cy])

####################################################################################################
# Give random rotation for the initial start position.
        rang = np.pi/4 * -rnum * random.random()
        if len(self.obstacles) > 0 and len(self.balls) > 0:
            points = np.concatenate((self.balls, self.obstacles))
        else:
            points = np.array(self.balls)

        if points.size > 0:
            points = points.reshape(-1,2)
            #points = np.add(points,[-axisdx, 0])
            theta = rang #rot_scale*rot*np.pi/180
            theta_0 = np.arctan2(points.T[1],points.T[0])

            ball_dist = np.linalg.norm(points, axis=1)
            rot_delta_unit_x = np.subtract(np.cos(theta_0), np.cos(np.add(theta_0,theta)))
            rot_delta_unit_y = np.subtract(np.sin(theta_0), np.sin(np.add(theta_0,theta)))
            rot_delta_unit = np.concatenate((rot_delta_unit_x.reshape(-1,1),rot_delta_unit_y.reshape(-1,1)),axis=1)
            ball_dist = np.concatenate((ball_dist.reshape(-1,1),ball_dist.reshape(-1,1)),axis=1)
            rot_delta = np.multiply(ball_dist, rot_delta_unit)
            points = np.subtract(points, rot_delta)
            #points = np.add(points,[2, 0])

            self.balls = points[0:len(self.balls)]
            self.obstacles = points[len(self.balls):]

##################################################################################
        self.draw_state()

        return self.frame_gray

    def check_window_map(self, cx, cy):
        inscreen = True
        if cx < 0 or cx >= map_param["width"]:
            inscreen = False
        if cy < 0 or cy >= map_param["height"]:
            inscreen = False
        return inscreen

    def check_window_state(self, cx, cy):
        inscreen = True
        if cx < 0 or cx >= simulator["width"]:
            inscreen = False
        if cy < 0 or cy >= simulator["height"]:
            inscreen = False
        return inscreen

    def draw_debug_frame(self, frame):
        frame_debug = np.zeros((simulator["height"]*debug_scale,simulator["width"]*debug_scale,3), np.uint8)
# Draw the obstacles and balls
        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(frame_debug,(i*debug_scale,j*debug_scale),((i+1)*debug_scale-1,(j+1)*debug_scale-1),(255,255,0),-1)
                if frame[i][j] == self._params["Map.data.ball"] or frame[i][j] == self._params["Map.data.ball_track"]:
                    cv2.rectangle(frame_debug,(i*debug_scale,j*debug_scale),((i+1)*debug_scale-1,(j+1)*debug_scale-1),(0,255,0),-1)



################################################################################################
# Draw the high_reward region
        cv2.rectangle(frame_debug,((simulator["center"]+high_reward_region_y[0]-1)*debug_scale-1,(simulator["height"]-high_reward_region_x[-1]-1)*debug_scale+1),\
                    ((simulator["center"]+high_reward_region_y[-1])*debug_scale,(simulator["height"]-high_reward_region_x[0])*debug_scale),(0,255,0),-1)
# Draw the low_reward region
        cv2.rectangle(frame_debug,((simulator["center"]+low_reward_region_y[0]-1)*debug_scale-1,(simulator["height"]-low_reward_region_x[-1]-1)*debug_scale+1),\
                    ((simulator["center"]+low_reward_region_y[-1])*debug_scale,(simulator["height"]-low_reward_region_x[0])*debug_scale),(0,160,0),-1)
# Draw the penalty region1(One on the right)
        cv2.rectangle(frame_debug,((simulator["center"]+penalty_region1_y[0])*debug_scale,(simulator["height"]-penalty_region1_x[-1]-1)*debug_scale+1),\
                    ((simulator["center"]+penalty_region1_y[-1]+1)*debug_scale,(simulator["height"]-penalty_region1_x[0])*debug_scale),(0,0,255),-1)

# Draw the penalty region2(One on the left)
        cv2.rectangle(frame_debug,((simulator["center"]+penalty_region2_y[0])*debug_scale-1,(simulator["height"]-penalty_region2_x[-1]-1)*debug_scale+1),\
                    ((simulator["center"]+penalty_region2_y[-1]+1)*debug_scale,(simulator["height"]-penalty_region2_x[0])*debug_scale),(0,0,255),-1)





# Draw the penalty region3(One on the bottom, car body and dustpan part)
        #cv2.rectangle(frame_debug,((simulator["center"]+penalty_region3_y[0])*debug_scale-1,(simulator["height"]-penalty_region3_x[-1]-1)*debug_scale+1),\
        #            ((simulator["center"]+penalty_region3_y[-1]+1)*debug_scale,(simulator["height"]-penalty_region3_x[0])*debug_scale-1),(0,0,255),-1)

# Draw the box inside penalty region3(To separate car body and dustpan part)
        #cv2.rectangle(frame_debug,((simulator["center"]+penalty_region3_y[0])*debug_scale-1,(simulator["height"]-penalty_region3_x[-1]-1)*debug_scale+1),\
        #            ((simulator["center"]+penalty_region3_y[-1]+1)*debug_scale,(simulator["height"]-penalty_region3_x[0]-5)*debug_scale-1),(0,0,255),-1)

# Draw the car body region
        #cv2.rectangle(frame_debug,((simulator["center"]+penalty_region3_y[0]+2)*debug_scale,(simulator["height"]-penalty_region3_x[-1]+1)*debug_scale+1),\
        #            ((simulator["center"]+penalty_region3_y[-1]-1)*debug_scale,(simulator["height"]-penalty_region3_x[0])*debug_scale-1),(0,0,0),-1)

# Draw the blue box at the center
        #cv2.rectangle(frame_debug,(center_of_rotation_y*debug_scale-1,(center_of_rotation_x-1)*debug_scale+1),\
        #            ((center_of_rotation_y+1)*debug_scale,center_of_rotation_x*debug_scale-1),(255,0,0),-1)


# Draw the trace

################################################################################################

# Draw vertical and horizontal grids
        for i in range(1,simulator["width"]):
            cv2.line(frame_debug,(i*debug_scale,0),(i*debug_scale,simulator["height"]*debug_scale-1),(128,128,128),1)
            cv2.line(frame_debug,(0,i*debug_scale),(simulator["width"]*debug_scale-1,i*debug_scale),(128,128,128),1)

# Draw FOV line
        #cv2.line(frame_debug,((simulator["center"]+ball_blind_bias - lidardy)*debug_scale - 4, (simulator["height"] - lidardx - axisdx)*debug_scale-1),\
        #                    ((simulator["width"]+ 10 - lidardy)*debug_scale-4,(simulator["height"]-int(ball_blind_ratio*(simulator["center"]-1-ball_blind_bias + 10)) - lidardx - axisdx)*debug_scale),(128,128,128),2)
        #cv2.line(frame_debug,((simulator["center"]+ball_blind_bias - lidardy)*debug_scale - 4,(simulator["height"] - lidardx - axisdx)*debug_scale-1),\
        #                    (2*((simulator["center"]+ball_blind_bias - lidardy)*debug_scale - 4)-((simulator["width"]+ 10 - lidardy)*debug_scale-4),(simulator["height"]-int(ball_blind_ratio*(simulator["center"]-1-ball_blind_bias + 10)) - lidardx - axisdx)*debug_scale),(128,128,128),2)

        cv2.line(frame_debug,((simulator["center"]+ball_blind_bias)*debug_scale,simulator["height"]*debug_scale-1),\
                            (simulator["width"]*debug_scale-1,(simulator["height"]-int(ball_blind_ratio*(simulator["center"]-1-ball_blind_bias)))*debug_scale),(128,128,128),2)
        cv2.line(frame_debug,((simulator["center"]-ball_blind_bias+1)*debug_scale,simulator["height"]*debug_scale-1),\
                            (0,(simulator["height"]-int(ball_blind_ratio*(simulator["center"]-1-ball_blind_bias)))*debug_scale),(128,128,128),2)


# Draw the Red box(Robot Padding)
        #cv2.rectangle(frame_debug,((simulator["center"]-3)*debug_scale-1,(simulator["height"]-1)*debug_scale-2),\
        #            ((simulator["center"]+3)*debug_scale,simulator["height"]*debug_scale-1),(0,0,255),2)

# Write 'Score' on top right
        cv2.putText(frame_debug,"Score "+str(self.score), (int(simulator["width"]*debug_scale*0.65),int(simulator["width"]*debug_scale*0.05)), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
# Write 'Step' on top left
        cv2.putText(frame_debug,"Step "+str(self.iter), (int(simulator["width"]*debug_scale*0.05),int(simulator["width"]*debug_scale*0.05)), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
####################################################################################################
# Write 'Balls Picked' on top below left
        cv2.putText(frame_debug,"Balls Picked "+str(len(self.balls_picked)), (int(simulator["width"]*debug_scale*0.05),int(simulator["width"]*debug_scale*0.1)), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
# Write 'Balls Left' on top below right
        cv2.putText(frame_debug,"Balls Left "+str(len(self.balls)), (int(simulator["width"]*debug_scale*0.65),int(simulator["width"]*debug_scale*0.1)), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))

################################################################################################
        return frame_debug

#################################################################################################
    def draw_global_trace(self, frame):
        global_trace = np.zeros((map_param["height"],map_param["width"],3), np.uint8)
# Draw the obstacles and balls
        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(frame_debug,(i*debug_scale,j*debug_scale),((i+1)*debug_scale-1,(j+1)*debug_scale-1),(255,255,0),-1)
                if frame[i][j] == self._params["Map.data.ball"] or frame[i][j] == self._params["Map.data.ball_track"]:
                    cv2.rectangle(frame_debug,(i*debug_scale,j*debug_scale),((i+1)*debug_scale-1,(j+1)*debug_scale-1),(0,255,0),-1)


# Draw the trace


# Draw vertical and horizontal grids
        for i in range(1,simulator["width"]):
            cv2.line(frame_debug,(i*debug_scale,0),(i*debug_scale,simulator["height"]*debug_scale-1),(128,128,128),1)
            cv2.line(frame_debug,(0,i*debug_scale),(simulator["width"]*debug_scale-1,i*debug_scale),(128,128,128),1)

# Write 'Score' on top right
        cv2.putText(frame_debug,"Score "+str(self.score), (int(simulator["width"]*debug_scale*0.65),int(simulator["width"]*debug_scale*0.05)), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))

        return global_trace

################################################################################################





    def draw_state_gray(self):
        gray_color = {"ball":255, "wall":100, "robot":200, "robot_padding":150}
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray,simulator["width"]*debug_scale_gray,1), np.uint8)

        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if self.frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(self.frame_gray,(i*debug_scale_gray,j*debug_scale_gray),((i+1)*debug_scale_gray-1,(j+1)*debug_scale_gray-1),gray_color["wall"],-1)
                if self.frame[i][j] == self._params["Map.data.ball"] or self.frame[i][j] == self._params["Map.data.ball_track"]:
                    cv2.rectangle(self.frame_gray,(i*debug_scale_gray,j*debug_scale_gray),((i+1)*debug_scale_gray-1,(j+1)*debug_scale_gray-1),gray_color["ball"],-1)

        #cv2.rectangle(self.frame_gray,((simulator["center"]-2)*debug_scale_gray-1,(simulator["height"]-2)*debug_scale_gray+1),\
        #            ((simulator["center"]+3)*debug_scale_gray,simulator["height"]*debug_scale_gray-1),gray_color["robot_padding"],-1)
        #cv2.rectangle(self.frame_gray,(simulator["center"]*debug_scale_gray-1,(simulator["height"]-1)*debug_scale_gray+1),\
        #            ((simulator["center"]+1)*debug_scale_gray,simulator["height"]*debug_scale_gray-1),gray_color["robot"],-1)
##################################################################################################################################################
# Draw the high_reward region
        cv2.rectangle(self.frame_gray,((simulator["center"]+high_reward_region_y[0]-1)*debug_scale_gray-1,(simulator["height"]-high_reward_region_x[-1]-1)*debug_scale_gray+1),\
                    ((simulator["center"]+high_reward_region_y[-1])*debug_scale_gray,(simulator["height"]-high_reward_region_x[0])*debug_scale_gray),gray_color["ball"],-1)
# Draw the low_reward region
        cv2.rectangle(self.frame_gray,((simulator["center"]+low_reward_region_y[0]-1)*debug_scale_gray-1,(simulator["height"]-low_reward_region_x[-1]-1)*debug_scale_gray+1),\
                    ((simulator["center"]+low_reward_region_y[-1])*debug_scale_gray,(simulator["height"]-low_reward_region_x[0])*debug_scale_gray),gray_color["robot_padding"],-1)
# Draw the penalty region1(One on the right)
        cv2.rectangle(self.frame_gray,((simulator["center"]+penalty_region1_y[0])*debug_scale_gray,(simulator["height"]-penalty_region1_x[-1]-1)*debug_scale_gray+1),\
                    ((simulator["center"]+penalty_region1_y[-1]+1)*debug_scale_gray-1,(simulator["height"]-penalty_region1_x[0])*debug_scale_gray),gray_color["robot"],-1)

# Draw the penalty region2(One on the left)
        cv2.rectangle(self.frame_gray,((simulator["center"]+penalty_region2_y[0])*debug_scale_gray,(simulator["height"]-penalty_region2_x[-1]-1)*debug_scale_gray+1),\
                    ((simulator["center"]+penalty_region2_y[-1]+1)*debug_scale_gray-1,(simulator["height"]-penalty_region2_x[0])*debug_scale_gray),gray_color["robot"],-1)

# Draw the penalty region3(One on the bottom, car body and dustpan part)
        #cv2.rectangle(self.frame_gray,((simulator["center"]+penalty_region3_y[0])*debug_scale_gray-1,(simulator["height"]-penalty_region3_x[-1]-1)*debug_scale_gray+1),\
        #            ((simulator["center"]+penalty_region3_y[-1]+1)*debug_scale_gray,(simulator["height"]-penalty_region3_x[0])*debug_scale_gray-1),gray_color["robot_padding"],-1)

# Draw the car body region
        #cv2.rectangle(self.frame_gray,((simulator["center"]+penalty_region3_y[0]+2)*debug_scale_gray,(simulator["height"]-penalty_region3_x[-1]+1)*debug_scale_gray+1),\
        #            ((simulator["center"]+penalty_region3_y[-1]-1)*debug_scale_gray,(simulator["height"]-penalty_region3_x[0])*debug_scale_gray-1),gray_color["robot"],-1)

# Draw the center of rotation
        # cv2.rectangle(self.frame_gray,(center_of_rotation_y*debug_scale_gray-1,(center_of_rotation_x-1)*debug_scale_gray+1),\
        #             ((center_of_rotation_y+1)*debug_scale_gray,center_of_rotation_x*debug_scale_gray-1),(255,0,0),-1)

##################################################################################################################################################








        return self.frame_gray

    def draw_state(self):
        store_state_inaccurate = self.state_inaccurate
        self.frame = np.zeros((simulator["height"],simulator["width"],1), np.uint8)
        for obstacle in self.obstacles:
############################################################################################################
# Make the walls randomly distributed(within 1 or 2 pixels) to mimic the SLAM environment with LIDAR.
            cx = simulator["center"] - int(round(1.0*obstacle[1]/trans_scale))
            cy = simulator["height"] - 1 - int(round(1.0*obstacle[0]/trans_scale))
            cx = cx + int(0.8 * random.random()*map_param["center"]*(0.1*cx*cx/map_param["center"]/map_param["center"] - 0.05))
            cy = cy + int(0.8 * random.random()*map_param["center"]*(0.1*cy*cy/map_param["center"]/map_param["center"] - 0.05))
##############################################################################################################
            if self.check_window_state(cx, cy):
                self.frame[cx][cy] = self._params["Map.data.obstacle"]
        for ball in self.balls:
            if self.state_blink == False or random.random() > (0.3 + ball[0]/3.0/map_param["center"]):
                if ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                    ball_x = ball[0]
                    ball_y = ball[1]
                    #if self.state_inaccurate:
                    #    ball_x = ball_x + random.random()*map_param["center"]*(0.1*ball_x*ball_x/map_param["center"]/map_param["center"] - 0.05)
                    #    ball_y = ball_y + random.random()*map_param["center"]*(0.1*ball_x*ball_x/map_param["center"]/map_param["center"] - 0.05)
                    if ball_x ** 2 + ball_y ** 2 <= 10 ** 2:
                        self.state_inaccurate = False

                    #This one is not recommended. Error smaller than real situation. But note that in real situation, the error of ball location gets smaller as robot approaches it.
                    if self.state_inaccurate:
                        ball_x = ball_x + 0.5 * random.random()*map_param["center"]*(0.1*ball_x*ball_x/map_param["center"]/map_param["center"] - 0.05)
                        ball_y = ball_y + 0.5 * random.random()*map_param["center"]*(0.1*ball_x*ball_x/map_param["center"]/map_param["center"] - 0.05)

                    self.state_inaccurate = store_state_inaccurate
                    cx = simulator["center"] - int(round(1.0*ball_y/trans_scale))
                    cy = simulator["height"] - 1 - int(round(1.0*ball_x/trans_scale))
                    if self.check_window_state(cx, cy):
                        self.frame[cx][cy] = self._params["Map.data.ball"]
        self.frame[simulator["center"]][simulator["height"]-1] = 255

        self.draw_state_gray()

        return self.frame

    def get_reward(self, action):
        reward = 0
        balls_temp = []
##########################################################################################################
        wall_too_close = False
        for obstacle in self.obstacles:
            # if int(abs(1.0*obstacle[0]/trans_scale)) <= 0 and int(abs(1.0*obstacle[1]/trans_scale)) <= 0:
            cx = obstacle[0]
            cy = obstacle[1]
            if cx ** 2 + cy ** 2 < 7 ** 2:
                wall_too_close = True
                #print(cx, cy, cx ** 2 + cy ** 2)
                #print("=========================")
        if wall_too_close:
            #print("Walls too close")
            reward = reward - 6

        if self.ball_inscreen_history >= 25:
            reward = reward - 6
        if self.wall_approaching_history >= 20:
            reward = reward - 6
        #print(self.cont_rot_history)
        if self.cont_rot_history >= 40:
            reward = reward - 6
            #print("Rotated too much")
        for i, ball in enumerate(self.balls):
##########################################################################################################
            #cx = int(round(1.0*ball[0]/trans_scale))
            #cy = int(round(abs(1.0*ball[1]/trans_scale)))
            #if  cx < reward_region_x and cx >= 0 and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
            #    if cy <= reward_region_y[0]:
            #        reward = reward + 7
            #    elif cy <= reward_region_y[1]:
            #        reward = reward + 3
            #    elif cy <= reward_region_y[2]:
            #        reward = reward + 1
            #        if len(self.balls_prev) > 0:
            #            if int(round(1.0*self.balls_prev[i][0]/trans_scale)) < reward_region_x:
            #                reward = reward - 2
            #    else:
            #        balls_temp.append(ball)
            #else:
            #    balls_temp.append(ball)
            cx = int(round(1.0*ball[0]/trans_scale))
            cy = int(round(1.0*ball[1]/trans_scale))
            #if cx < reward_region_x[-1] and cx >= 0 and cy >= 0 and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
            if cx < low_reward_region_x[-1] + 1 and cx >= low_reward_region_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                if low_reward_region_y[0] <= cy <= low_reward_region_y[-1]:
                    self.balls_in_contact = False
                    reward = reward + 50
                    if len(self.balls_prev) > 0:
                        #if int(round(1.0*self.balls_prev[i][0]/trans_scale)) <= low_reward_region_x[-1]:
                        #    reward = reward - 80
                        if low_reward_region_x[0] <= int(round(1.0*self.balls_prev[i][0]/trans_scale)) < low_reward_region_x[-1]+1:
                            reward = reward - 60

            if cx < penalty_region1_x[-1] + 1 and cx >= penalty_region1_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                if penalty_region1_y[0] - 4 <= cy < penalty_region1_y[-1] - 3:
                    reward = reward - 250
            if cx < penalty_region2_x[-1] + 1 and cx >= penalty_region2_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                if abs(penalty_region2_y[-1])  <= cy < abs(penalty_region2_y[0]) + 1:
                    reward = reward - 250

            if cx < high_reward_region_x[-1] + 1 and cx >= high_reward_region_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                if high_reward_region_y[0] <= cy <= high_reward_region_y[-1]:
                    self.balls_in_contact = False
                    if len(self.balls) == 1:
                        reward = reward + 600
                        self.ball_inscreen_history = 0
                    else:
                        reward = reward + 200
                        self.ball_inscreen_history = 0

###############################################################################################
                    self.balls_picked.append(ball)
###############################################################################################
                    if len(self.balls_prev) > 0:
                        if int(round(1.0*self.balls_prev[i][0]/trans_scale)) < high_reward_region_x[-1] + 1:
                            reward = reward - 10
                else:
                    balls_temp.append(ball)
            else:
                balls_temp.append(ball)
############################################################################################

        balls_inscreen = []
        for ball in balls_temp:
            if ball[0] >= ball_blind_ratio * (abs(1.0*ball[1]) - ball_blind_bias)\
                and abs(1.0*ball[1]) <= map_param["center"] and abs(1.0*ball[0]) < map_param["height"]:
                balls_inscreen.append(ball)

        self.balls = balls_temp
        # if self.debug_flag:
        #     print("balls length : "+str(len(balls_temp))+"  score : "+str(self.score)+"  screen_flag : "+str(self.ball_inscreen_flag))
###################################################################################
        #if action in range(10):
        if action in range(4):
            if len(balls_inscreen) == 0:
                self.ball_inscreen_flag = self.ball_inscreen_flag + 1
                self.ball_inscreen_history = 0
            else:
                self.ball_inscreen_flag = 0
                self.ball_inscreen_history = self.ball_inscreen_history + 1
        #if len(balls_temp) == 0 or self.iter > max_iter or self.ball_inscreen_flag >= 10:
        #    self.done = True
        #print(self.cont_rot_history)
        wall_approaching = False
        if action in range(4):
            for obstacle in self.obstacles:
            # if int(abs(1.0*obstacle[0]/trans_scale)) <= 0 and int(abs(1.0*obstacle[1]/trans_scale)) <= 0:
                if obstacle[0] ** 2 + obstacle[1] ** 2 < 8.0 ** 2:
                    wall_approaching = True
            if wall_approaching == True:
                self.wall_approaching_history += 1
                #print("Watch out for walls")
            else:
                self.wall_approaching_history = 0
        #print(self.wall_approaching_history)

        if len(balls_temp) == 0 or self.iter > max_iter or self.ball_inscreen_flag >= 8 * math.ceil(360/rot_scale) + 86:
        #if len(balls_temp) == 0 or self.iter > max_iter or self.ball_inscreen_flag >= math.ceil(360/rot_scale)+ 6:
            self.done = True
##########################################################################################
        if self.done:
            self.episode_rewards.append(self.score)
            if self.write_flag:
                self.video.release()
                print ("video saved")
        if action == 0:
            if self.cont_rot_history > 24:
                reward = reward + 3
                #print("Out of loop!")
            if len(balls_inscreen) == 0:
                reward = reward - 4
            else:
                reward = reward - 1
        if action == 1 or action == 2:
            self.cont_rot_history += 1
            if len(balls_inscreen) == 0:
                if action == 1:
                    reward = reward - 2 #10
                else:
                    reward = reward - 1
            else:
                reward = reward - 1
        elif action == 0 or action == 3:
            self.cont_rot_history = 0
        #print(self.move_back_history)
        if action == 3:
            if 5 <= self.move_back_history:
                reward = reward - 30
                #print("Moved back too much")
            else:
                reward = reward - 8
        if action == -1:
##################################################################
# If there's no ball in the sight of robot and do nothing, -10 points
            if len(balls_inscreen) == 0:
                return  - 4
#################################################################
            else:
                return - 2
        else:
            return reward

    def step(self, action):
        # print "action "+str(action)
###################################################################################
        #if action in range(10):
        #    self.iter = self.iter + 1
        if action in range(4):
            self.iter = self.iter + 1
####################################################################################

        del_x, del_y, rot = 0, 0, 0

#############################################################################
# If you change actions, you need to change the lines under
# "if key == ord('q') or tk.done == True:" too
        #if action == 0: # forward
        #    del_x, del_y = -1, 0
        #elif action == 1: # forward right
        #    del_x, del_y = -1, 1
        #elif action == 2: # right
        #    del_x, del_y = 0, 1
        #elif action == 3: # backward right
        #    del_x, del_y = 1, 1
        #elif action == 4: # backward
        #    del_x, del_y = 1, 0
        #elif action == 5: # bacward left
        #    del_x, del_y = 1, -1
        #elif action == 6: # left
        #    del_x, del_y = 0, -1
        #elif action == 7: # forward left
        #    del_x, del_y = -1, -1
        #elif action == 8: # turn left
        #    rot = -1
        #elif action == 9: # turn right
        #    rot = 1
        #else:
        #    del_x, del_y, rot_x = 0, 0, 0
        if action == 0: # forward
            del_x, del_y = -1, 0
        elif action == 1: # rotate 'rot_scale' degrees counterclockwise
            rot = -1
        elif action == 2: # rotate 'rot_scale' degrees clockwise
            rot = 1
        elif action == 3:
            del_x, del_y = 1, 0
########################################################################################
        else:
            del_x, del_y, rot_x = 0, 0, 0
###################################################################


        balls_temp = []
        obstacles_temp = []

        del_x = del_x * trans_scale
        del_y = del_y * trans_scale

        if len(self.balls) > 0:
            balls_temp = np.add(self.balls, [del_x ,del_y])

        if len(self.obstacles) > 0:
            obstacles_temp = np.add(self.obstacles, [del_x ,del_y])
#########################################################################
        #if action == 8 or action == 9:
        if action == 1 or action == 2:
############################################################################
            if len(self.obstacles) > 0 and len(balls_temp) > 0:
                points = np.concatenate((balls_temp, obstacles_temp))
            else:
                points = np.array(balls_temp)

            if points.size > 0:
                points = points.reshape(-1,2)
                points = np.add(points,[-axisdx, 0])
                theta = rot_scale*rot*np.pi/180
                theta_0 = np.arctan2(points.T[1],points.T[0])

                ball_dist = np.linalg.norm(points, axis=1)
                rot_delta_unit_x = np.subtract(np.cos(theta_0), np.cos(np.add(theta_0,theta)))
                rot_delta_unit_y = np.subtract(np.sin(theta_0), np.sin(np.add(theta_0,theta)))
                rot_delta_unit = np.concatenate((rot_delta_unit_x.reshape(-1,1),rot_delta_unit_y.reshape(-1,1)),axis=1)
                ball_dist = np.concatenate((ball_dist.reshape(-1,1),ball_dist.reshape(-1,1)),axis=1)
                rot_delta = np.multiply(ball_dist, rot_delta_unit)
                points = np.subtract(points, rot_delta)
                points = np.add(points,[axisdx, 0])

                balls_temp = points[0:len(self.balls)]
                obstacles_temp = points[len(self.balls):]

        enable_move = True
        rotation_wall = False
####################################################################################################################
        # for ball in balls_temp:
        #     cx = round(ball[0])
        #     cy = round(ball[1])
        #     if cx < penalty_region1_x[-1] + 1 and cx >= penalty_region1_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
        #         if penalty_region1_y[0] - 4 <= cy < penalty_region1_y[-1] - 3:
        #             enable_move = False
        #             self.balls_in_contact = True
        #     if cx < penalty_region2_x[-1] + 1 and cx >= penalty_region2_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
        #         if abs(penalty_region2_y[-1])  <= cy < abs(penalty_region2_y[0]) + 1:
        #             enable_move = False
        #             self.balls_in_contact = True

        for obstacle in obstacles_temp:
            # if int(abs(1.0*obstacle[0]/trans_scale)) <= 0 and int(abs(1.0*obstacle[1]/trans_scale)) <= 0:
            if abs(1.0*obstacle[0]) < 4.0 and abs(1.0*obstacle[1]) < 4.0:
                enable_move = False
                #print("move impossible")
                if action == 1 or action == 2:
                    rotation_wall = True
        if rotation_wall == True:
            self.cont_rot_history += 1



        reward = 0
        #print(self.move_back_history)
        if action == 3:
            self.move_back_history += 1
            if self.wall_approaching_history == 0:
                #enable_move = False
                if 5 <= self.move_back_history:
                    reward = reward - 30
                    #print("Moved back too much")
            for ball in balls_temp:
                cx = math.floor(ball[0])
                cy = round(ball[1])
                if cx ** 2 + cy **2 < 6 ** 2:
                    enable_move = True
                    if cx < low_reward_region_x[-1] + 2 and cx >= low_reward_region_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
                        if low_reward_region_y[0] <= cy <= low_reward_region_y[-1]:
                            pass
                            #enable_move = False
        # if action == 1 or action == 2:
        #     for ball in balls_temp:
        #         cx = round(ball[0])
        #         cy = round(ball[1])
        #         if cx < low_reward_region_x[-1] + 2 and cx >= low_reward_region_x[0] and ball[0] >= int(ball_blind_ratio*(abs(1.0*ball[1])-ball_blind_bias)):
        #             if low_reward_region_y[0] <= cy <= low_reward_region_y[-1]:
        #                 enable_move = False

            if self.balls_in_contact == True:
                enable_move = True
            self.balls_in_contact = False

        if self.cont_rot_history >= 32:
            reward = reward - 6

        elif action == 0 or action == 1 or action == 2:
            self.move_back_history = 0

        #print(self.balls_in_contact)
        if enable_move == True:
            self.balls_in_contact = False
####################################################################################################################

        if enable_move:
            self.balls = balls_temp
            reward = self.get_reward(action) + reward
            self.obstacles = obstacles_temp
            self.draw_state()
            self.balls_prev = self.balls
        else:
            #reward = self.get_reward(-1)
            reward = self.get_reward(-1) - 12 + reward #-1

        self.score = self.score + reward

        if self.write_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            self.video.write(frame_debug)

        if self.debug_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            cv2.imshow("frame_debug", frame_debug)
            cv2.imshow("frame_debug_gray", self.frame_gray)
            #cv2.imshow("global_trace", self.global_trace)
            cv2.waitKey(100)

        return self.frame_gray, reward, self.done

    def get_total_steps(self):
        return self.iter

    def get_episode_rewards(self):
        return self.episode_rewards

    def action_space_sample(self):
        index = int(1.0*random.random()*10)
        return self.action_space[index]

    def callback():
        return

if __name__ == '__main__':
    tk = Task(debug_flag=True, test_flag=False, state_blink=True, state_inaccurate=True)
    tk.reset()

    action = -1
    while(1):
        tk.step(action)
        key = cv2.waitKey(300)&0xFF
        action = -1
################################################################################
        #if key == ord('q') or tk.done == True:
        #    break;
        #elif key == ord('w'):
        #    action = 0
        #elif key == ord('d'):
        #    action = 2
        #elif key == ord('s'):
        #    action = 4
        #elif key == ord('a'):
        #    action = 6
        #elif key == ord('z'):
        #    action = 8
        #elif key == ord('c'):
        #    action = 9
        if key == ord('q') or tk.done == True:
            print("Your score is: {0.score}".format(tk))
            break;
        elif key == ord('w'):
            action = 0
        elif key == ord('a'):
            action = 1
        elif key == ord('d'):
            action = 2
        elif key == ord('s'):
            action = 3
##################################################################################

    print("shutdown")
    # cv2.destroyAllWindows()
