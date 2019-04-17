import rospy
import time
import tf
import geometry_msgs
import tf_conversions
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from std_msgs.msg import Empty
import numpy as np

model_states = {}


def eq(x, y, err = 0.01):
	return abs(x-y) < err


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def model_states_callback(msg):
	global model_states
	models = msg.name
	poses = msg.pose
	twists = msg.twist
	for i in range(len(models)):
		model_states[models[i]] = (poses[i], twists[i])


def reset_ball():
	pose = Pose()
	pose.position.x = 0
	pose.position.y = 1
	pose.position.z = 2
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 0

	state = ModelState()
	state.model_name = "ball"
	state.pose = pose

	rospy.wait_for_service("/gazebo/set_model_state")
	
	try:
		ball = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		ret = ball(state)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def quaternion_to_euler(quat):
	quaternion = (quat.x, quat.y, quat.z, quat.w)
	return tf.transformations.euler_from_quaternion(quaternion)


def euler_to_quaternion(euler):
	return tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])


def adjust_block(err):
	rospy.wait_for_service("/gazebo/set_model_state")
	global model_states
	pose = Pose()
	pose.position = model_states["keva_plank_6"][0].position
	quat = model_states["keva_plank_6"][0].orientation
	euler = quaternion_to_euler(quat)
	yaw = euler[2]
	# just easy stuff
	yaw += -0.2 * err
	pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], yaw))
	state = ModelState()
	state.model_name = "keva_plank_6"
	state.pose = pose
	try:
		block = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		ret = block(state)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



def main():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
	start = time.time()
	iteration_going = True
	prev_dir = [0, 0, -1]
	reset_ball()
	time.sleep(0.5)
	while not rospy.is_shutdown():
		chp = time.time()
		if model_states.get('ball'):
			ball_pos = model_states.get('ball')[0].position
			ball_ori = model_states.get('ball')[0].orientation
			ball_vel_ros = model_states.get('ball')[1].linear
			ball_vel = [ball_vel_ros.x, ball_vel_ros.y, ball_vel_ros.z]
			angle_diff = angle_between(prev_dir, ball_vel)
			if angle_diff > 0.05:
				print ball_vel
				prev_dir = unit_vector(ball_vel)
			if ball_pos.x <= 0.1:
				iteration_going = True
			if ball_pos.x >= 2 and iteration_going:
				if ball_pos.y >= -3.1 and ball_pos.y <= -2.9:
					print "Done"
					print quaternion_to_euler(model_states.get('keva_plank_6')[0].orientation)
					break
				else:
					iteration_going = False
					err = ball_pos.y+3
					adjust_block(err)
					start = chp
					reset_ball()

		if chp-start > 10:
			start = chp
			err = 3
			adjust_block(err)
			reset_ball()


if __name__ == "__main__":
	main()
