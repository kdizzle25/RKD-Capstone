"""
    You will use this script for the final demo.
    Implement your code flow here.
    Your main fucntion can take in different command line arguments to run different parts of your code.
"""
import argparse
from motion_planner import *
from robot import *
from utils import *
from calibrate_workspace import *
import time
def oogabooga():
	if __name__ == '__main__':
		parser = argparse.ArgumentParser()
		parser.add_argument('--time', '-t', type=float, default=10)
		parser.add_argument('--open_gripper', '-o', action='store_true')
		args = parser.parse_args()

		print('Starting robot')
		fa = FrankaArm()
		if args.open_gripper:
			fa.open_gripper()
		print('Applying 0 force torque control for {}s'.format(args.time))
		fa.run_guide_mode(args.time)
		print(fa.get_joints())
		return fa.get_joints()

# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3

# Call the program with passed arguments
generator = TrajectoryGenerator()
follower = TrajectoryFollower()
follower.fa.reset_pose()
follower.fa.reset_joints()
    
p1 = oogabooga()
p2 = oogabooga()
p3 = oogabooga()

follower.fa.reset_pose()
follower.fa.reset_joints()

start = follower.fa.get_joints()
traj1 = generator.interpolate_joint_trajectory(start, p1)
follower.follow_joint_trajectory(traj1)
follower.fa.close_gripper()
time.sleep(5)
start = follower.fa.get_joints()
traj2 = generator.interpolate_joint_trajectory(start, p2)
follower.follow_joint_trajectory(traj2)
time.sleep(5)
start = follower.fa.get_joints()
traj3 = generator.interpolate_joint_trajectory(start, p3)
follower.follow_joint_trajectory(traj3)
follower.fa.open_gripper()

