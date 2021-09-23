#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list 
from constants import *


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        group_name = "arm_motion_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")

        print('Available Commands: ' + AVAILABLE_COMMANDS)

        
        # Misc variables
        self.box_name = ""
        self.robot = robot
        # self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.data = {}

    def save_new_trajectory(self):
        trajectory_name = input('Please enter a trajectory name')
        sequence_list = []
        move_group = self.move_group

        def append_trajectory():
            joint_goal = move_group.get_current_joint_values()
            print('Joint goal: ' + str(joint_goal))
            sequence_list.append(joint_goal)
        
        print('Press save to append current joint_values, stop to stop recording')
        input_command = 'Start'
        while (input_command != 'stop'):
            input_command = str(input())
            if (input_command == 'save'):
                append_trajectory()
            else:
                print('You keyed in an invalid command. Please try again')
        print('Recording stopped and trajectory will now be saved')
        print('Sequence List:' + str(sequence_list))
        self.data[trajectory_name] = sequence_list

        
    def go_to_joint_state(self):
        move_group = self.move_group
        print('IM IN')
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def plan_cartesian_path(self, trajectory_name):
        print('tn: ' + trajectory_name)
        waypoints = self.data[trajectory_name]
        print('WAYPOINTS : ')
        print(waypoints)
        print(type(waypoints))

        cpoint = []
        for point in waypoints:
            cpoint = point
            print('Point: ' + str(point) )
            self.move_group.go(point, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(point, current_joints, 0.01)
        


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def planner(self):
        indata = {}
        backup_data = {}
        with open('planner_data.json','r+') as file:
            try:
                indata = json.load(file)
                print('indata: ')
                print(indata)
                self.data = indata
                backup_data = indata
            except Exception as e:
                print('Error reading data: ' + str(e))
        while True:
            try:
                x = str(input('Please Enter a Command\n'))
                if (x == 'exit'):
                    break
                elif (x == 'ss' or x == 'save sequence'):
                    self.save_new_trajectory()
                elif (x == 'gp' or x == 'go pose'):
                    input("============ Press `Enter` to execute a movement using a joint state goal ...")
                    print(json.dumps(self.data))
                    self.go_to_joint_state()
                elif (x == 'pt' or x == 'plan traj'):
                    y = input('=========== Please enter trajectory name which you want to plan....')
                    self.plan_cartesian_path(y)
                elif (x == 'se' or x == 'save exit'):
                    with open('planner_data.json', 'w') as file:
                        try:
                            json.dump(self.data,file)
                            print("============ Saving JSON and terminating program!")
                            break
                        except:
                            print('Something went wrong while trying to save the JSON File')
                else:
                    print('Please print a valid input.')
            except ValueError:
                print('Please input a valid Command')
            except Exception as e:
                print('Something went wrong' + str(e))
        


def main():
    print(WELCOME)
    try:
        # input('============ Press Enter to Proceed Set up the Control Interface ============')
        # Initialise the Control Interface and the Terminal Command Parser
        controlInterface = MoveGroupPythonInterfaceTutorial()
        print("============ Started Planner ============")
        controlInterface.planner()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



if __name__ == "__main__":
    main()