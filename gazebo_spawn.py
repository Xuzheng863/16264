import rospy
import math
import geometry_msgs
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

plank_file = open('/home/zhengxu/model_editor_models/keva_plank/model.sdf', 'r')
ball_file = open('/home/zhengxu/model_editor_models/ball/model.sdf', 'r')

rospy.init_node('insert_object',log_level=rospy.INFO)

def spawn_model(model, center, yaw):
	initial_pose = Pose()
	initial_pose.position.x = center[0]
	initial_pose.position.y = center[1]
	initial_pose.position.z = center[2]
	initial_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(0, 0, yaw))

	if model == 'ball':
		sdff = ball_file.read()
	else:
		sdff = plank_file.read()

	rospy.wait_for_service('gazebo/spawn_sdf_model')
	spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
	spawn_model_prox("keva_plank_7", sdff, "", initial_pose, "world")


def main():
	spawn_model('plank', [-2, -2, 0.2], math.pi/2)


if __name__ == "__main__":
	main()