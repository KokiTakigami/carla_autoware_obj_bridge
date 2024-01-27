import carla
import rclpy
from rclpy.node import Node
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from autoware_auto_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification, DetectedObjectKinematics, Shape

from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleInfo  # Assuming you want to publish vehicle information

from carla_autoware_obj_bridge import transform as trans


class CarlaROS2Publisher(Node):
    def __init__(self):
        super().__init__('carla_ros2_publisher')
        self.carla_client = carla.Client('localhost', 2000)  # Connect to the Carla server
        self.carla_client.set_timeout(2.0)
        self.world = self.carla_client.get_world()

        # ROS 2 publisher
        self.autoware_objects_pub = self.create_publisher(
            DetectedObjects, "/perception/object_recognition/detection/objects", 
            rclpy.qos.QoSProfile(depth=1)
            )
        # self.autoware_objects_pub = self.create_publisher(
        #     DetectedObjects, "/lidar_center_point/output/objects", 
        #     rclpy.qos.QoSProfile(depth=1)
        #     )

        # ROS 2 timer to periodically publish data
        self.create_timer(0.1, self.publish_carla_data)

    def publish_carla_data(self):
        # Get all actors (vehicles, pedestrians, etc.) in the world
        actors = self.world.get_actors()

        detected_objects = DetectedObjects()
        detected_objects.header.stamp = self.get_clock().now().to_msg()
        detected_objects.header.frame_id = 'map'

        for actor in actors:
            detected_object = DetectedObject()
            detected_object.existence_probability = 0.0
            
            # Check if the actor is a vehicle
            if 'vehicle' in actor.type_id or 'pedestrian' in actor.type_id:
                # classification
                autoware_classification = ObjectClassification()
                autoware_classification.probability = 1.0
                if 'vehicle' in actor.type_id:
                    autoware_classification.label = ObjectClassification.CAR
                else:
                    autoware_classification.label = ObjectClassification.PEDESTRIAN
                detected_object.classification.append(autoware_classification)

                # kinematics
                autoware_kinematics = DetectedObjectKinematics()
                autoware_kinematics.has_position_covariance = False
                autoware_kinematics.has_twist_covariance = False
                autoware_kinematics.has_twist = False
                autoware_kinematics.orientation_availability = DetectedObjectKinematics.UNAVAILABLE
                autoware_kinematics.pose_with_covariance.pose = trans.carla_transform_to_ros_pose(actor.get_transform())
                autoware_kinematics.twist_with_covariance.twist \
                    = trans.carla_velocity_to_ros_twist(
                        actor.get_velocity(),
                        actor.get_angular_velocity(),
                        actor.get_transform().rotation)
                detected_object.kinematics = autoware_kinematics
                
                # shape
                autoware_shape = Shape()
                autoware_shape.type = Shape.BOUNDING_BOX
                autoware_shape.dimensions.x = actor.bounding_box.extent.x * 2.0
                autoware_shape.dimensions.y = actor.bounding_box.extent.y * 2.0
                autoware_shape.dimensions.z = actor.bounding_box.extent.z * 2.0
                detected_object.shape = autoware_shape

                detected_objects.objects.append(detected_object)

        self.autoware_objects_pub.publish(detected_objects)


def main(args=None):
    rclpy.init(args=args)
    carla_ros2_publisher = CarlaROS2Publisher()
    rclpy.spin(carla_ros2_publisher)
    carla_ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
