import carla
import rclpy
from rclpy.node import Node

import ros_compatibility as roscomp

from std_msgs.msg import Header
from derived_object_msgs.msg import ObjectArray
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleInfo  # Assuming you want to publish vehicle information

from carla_autoware_obj_bridge import transform as trans


class CarlaROS2Publisher(Node):
    def __init__(self):
        super().__init__('carla_ros2_publisher')
        self.carla_client = carla.Client('localhost', 2000)  # Connect to the Carla server
        self.carla_client.set_timeout(2.0)
        self.world = self.carla_client.get_world()

        self.dummy_classification_age = 0

        # ROS 2 publisher
        self.carla_objects_publisher = self.create_publisher(
            ObjectArray, 
            "/carla/ego_vehicle/objects", 
            qos_profile=10)

        # ROS 2 timer to periodically publish data
        self.create_timer(0.1, self.publish_carla_data)

    def get_msg_header(self, frame_id=None, timestamp=None):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = frame_id
        header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)
        return header

    def get_classification(self, carla_actor):
        # print(carla_actor.type_id)
        if 'vehicle' in carla_actor.type_id:
            classification = Object.CLASSIFICATION_CAR
            if 'object_type' in carla_actor.attributes:
                if carla_actor.attributes['object_type'] == 'car':
                    classification = Object.CLASSIFICATION_CAR
                elif carla_actor.attributes['object_type'] == 'bike':
                    classification = Object.CLASSIFICATION_BIKE
                elif carla_actor.attributes['object_type'] == 'motorcycle':
                    classification = Object.CLASSIFICATION_MOTORCYCLE
                elif carla_actor.attributes['object_type'] == 'truck':
                    classification = Object.CLASSIFICATION_TRUCK
                elif carla_actor.attributes['object_type'] == 'other':
                    classification = Object.CLASSIFICATION_OTHER_VEHICLE
        elif 'pedestrian' in carla_actor.type_id:
            classification = Object.CLASSIFICATION_PEDESTRIAN
        else:
            classification = Object.CLASSIFICATION_UNKNOWN
        return classification

    def publish_carla_data(self):
        # Get all actors (vehicles, pedestrians, etc.) in the world
        actors = self.world.get_actors()

        world_snapshot = self.world.get_snapshot()
        timestamp = world_snapshot.timestamp.elapsed_seconds

        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="map", timestamp=timestamp)

        for actor in actors:
            obj = Object(header=self.get_msg_header(frame_id="map", timestamp=timestamp))

            # ID
            obj.id = actor.id
            # Pose
            obj.pose = trans.carla_transform_to_ros_pose(
                actor.get_transform()
                )
            # Twist
            # obj.twist = trans.carla_velocity_to_ros_twist(
            #     actor.get_velocity(),
            #     actor.get_angular_velocity()
            #     )
            obj.twist = trans.carla_velocity_to_ros_twist(
                actor.get_velocity(),
                actor.get_angular_velocity(),
                actor.get_transform().rotation)
            # Acceleration
            obj.accel = trans.carla_acceleration_to_ros_accel(
                actor.get_acceleration()
            )
            # Shape
            obj.shape.type = SolidPrimitive.BOX
            obj.shape.dimensions.extend([
                actor.bounding_box.extent.x * 2.0,
                actor.bounding_box.extent.y * 2.0,
                actor.bounding_box.extent.z * 2.0])
            # Classification if available in attributes
            # print(self.get_classification(actor), Object.CLASSIFICATION_CAR)
            if self.get_classification(actor) != Object.CLASSIFICATION_UNKNOWN:
                obj.object_classified = True
                obj.classification = self.get_classification(actor)
                obj.classification_certainty = 255
                # obj.classification_age = self.classification_age
                obj.classification_age = self.dummy_classification_age

            # currently only Vehicles and Walkers are added to the object array
            if obj.classification == Object.CLASSIFICATION_CAR:
                ros_objects.objects.append(obj)
            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                ros_objects.objects.append(obj)
        
        self.dummy_classification_age += 1
        # print(ros_objects)
        self.carla_objects_publisher.publish(ros_objects)


def main(args=None):
    rclpy.init(args=args)
    carla_ros2_publisher = CarlaROS2Publisher()
    rclpy.spin(carla_ros2_publisher)
    carla_ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
