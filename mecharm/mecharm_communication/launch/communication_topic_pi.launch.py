# <launch>
# 	<arg name="port" default="/dev/ttyUSB0" />
# 	<arg name="baud" default="115200" />

# 	<!-- Open communication service -->
# 	<node name="mecharm_services" pkg="mecharm_communication" type="mecharm_topics_pi.py" output="screen">
# 		<param name="port" type="string" value="$(arg port)" />
# 		<param name="baud" type="int" value="$(arg baud)" />
#     </node>
# </launch>

from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription



def generate_launch_description():
    # <arg name="port" default="/dev/ttyUSB0" />
    port_launch_arg = DeclareLaunchArgument("port", default_value="/dev/ttyAMA0")


    # <arg name="baud" default="115200" />
    baud_launch_arg = DeclareLaunchArgument("baud", default_value="1000000")


    # <node name="mecharm_services" pkg="mecharm_communication" type="mecharm_topics_pi.py" output="screen">
    # 	<param name="port" type="string" value="$(arg port)" />
    # 	<param name="baud" type="int" value="$(arg baud)" />
    # </node>
    mecharm_communication_node_with_parameters = Node(
        name="mecharm_services",
        package="mecharm_communication",
        executable="mecharm_topics_pi",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
        }]
    )


    # from launch import LaunchDescription
    return LaunchDescription([
        port_launch_arg,
        baud_launch_arg,
        mecharm_communication_node_with_parameters
    ])
