from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, LogInfo
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# see this link for any help with creating launch files: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

def get_main_node(debug_text=''):

	### GROUP IF WITH_WORLD_PARAMS
	group_if = GroupAction( 
							condition = LaunchConfigurationEquals('with_world_params', 'true'),
							actions= [
								Node( package='dhtt', executable='start_server', output='debug', prefix=[debug_text], emulate_tty=True ),
								Node( package='dhtt', executable='param_node', output='log', emulate_tty=True, parameters=[LaunchConfiguration('params_file')]),
							]
	 					  )

	### GROUP UNLESS WITH_WORLD_PARAMS
	group_unless = GroupAction( 
							condition = LaunchConfigurationEquals('with_world_params', 'false'),
							actions= [
								Node( package='dhtt', executable='start_server', output='debug', prefix=[debug_text], emulate_tty=True ),
								Node( package='dhtt', executable='param_node', output='log', emulate_tty=True),
							]
	 					  )

	return [group_if, group_unless]

def generate_launch_description():

	### ARGUMENTS
	with_world_params_arg = DeclareLaunchArgument( 'with_world_params', default_value=TextSubstitution(text='false') )
	params_file_arg = DeclareLaunchArgument( 'params_file' , default_value=TextSubstitution(text='') )
	debug_arg = DeclareLaunchArgument( 'debug', default_value=TextSubstitution(text='false') )

	debug_group = GroupAction(
							condition= LaunchConfigurationEquals('debug', 'true'),
							actions=get_main_node('gdbserver localhost:2159')
						  )

	no_debug_group = GroupAction(
									condition= LaunchConfigurationEquals('debug', 'false'),
									actions=get_main_node()
								)

	return LaunchDescription([ 
			with_world_params_arg,
			params_file_arg, 
			debug_arg, 
			debug_group, 
			no_debug_group
		])