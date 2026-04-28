from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

	return LaunchDescription([
		DeclareLaunchArgument("griplink_ip_address", description="IP address of the griplink that should be connected to", default_value="192.168.1.40"),
		DeclareLaunchArgument("griplink_network_port", description="The Griplink should be connected to this network port", default_value="10001"),
		Node(
			package='griplink',
			namespace='griplink_node',
			executable='griplink_node',
			name='griplink_node',
			parameters=[
				{"ip": LaunchConfiguration("griplink_ip_address")},
				{"port": LaunchConfiguration("griplink_network_port")}
			]
			# arguments=['--ros-args', '--log-level', 'DEBUG']
		)
	])