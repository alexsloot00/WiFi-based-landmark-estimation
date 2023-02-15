#!/usr/bin/env python3


import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from terminal_functions import start_gazebo, start_roscore


def spawn_model(model_name: str, model_path: str, pose: Pose) -> None:
    """Spawn any model into the Gazebo world."""
    with open(model_path, "r") as model_file:
        model_xml = model_file.read()
    try:
        spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_model_prox(model_name, model_xml, "", pose, "world")
        rospy.loginfo("Model spawned successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn SDF model service call failed: {e}")


def delete_model(model_name: str) -> None:
    """Delete any model from the Gazebo world."""
    try:
        delete_model_prox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        delete_model_prox(model_name)
        rospy.loginfo("Model deleted successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Delete model service call failed: {e}")


def main() -> None:
    """Test spawning a model into the Gazebo world."""
    start_roscore()
    start_gazebo()

    rospy.init_node("spawn_nexus_car")

    model_name = "nexus_car"
    model_path = "/home/alex/.gazebo/models/cafe_table/model.sdf"
    # model_path = os.path.join(os.path.dirname(sys.argv[0]), "model.sdf")

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0

    spawn_model(model_name, model_path, pose)


if __name__ == "__main__":
    main()
