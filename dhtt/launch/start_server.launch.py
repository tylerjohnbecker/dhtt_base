from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, LogInfo
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# see this link for any help with creating launch files: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

def get_main_node(debug_text=''):

	group_test = GroupAction(
								condition = LaunchConfigurationEquals('test', 'true'),
								actions= [
									Node( package='dhtt', executable='start_server', output='log', prefix=[debug_text], emulate_tty=True, arguments=['test'] ),
								]
							)

	group_no_test = GroupAction(
								condition = LaunchConfigurationEquals('test', 'false'),
								actions= [
									Node( package='dhtt', executable='start_server', output='log', prefix=[debug_text], emulate_tty=True, arguments=[] ),
								]
							)

	### GROUP IF WITH_WORLD_PARAMS
	group_if = GroupAction( 
							condition = LaunchConfigurationEquals('with_world_params', 'true'),
							actions= [
								group_test,
								group_no_test,
								Node( package='dhtt', executable='param_node', output='log', emulate_tty=True, parameters=[LaunchConfiguration('params_file')]),
							]
	 					  )

	### GROUP UNLESS WITH_WORLD_PARAMS
	group_unless = GroupAction( 
							condition = LaunchConfigurationEquals('with_world_params', 'false'),
							actions= [
								group_test,
								group_no_test,
								Node( package='dhtt', executable='param_node', output='log', emulate_tty=True),
							]
	 					  )

	return [group_if, group_unless]

def generate_launch_description():

	### ARGUMENTS
	with_world_params_arg = DeclareLaunchArgument( 'with_world_params', default_value=TextSubstitution(text='false'), description="If true then the params file given in params_file will be loaded into the main servers param server on startup. This is mostly unused at the moment." )
	params_file_arg = DeclareLaunchArgument( 'params_file' , default_value=TextSubstitution(text=''), description="Params file to load from. An example file is shown at dhtt/tests/test_world_states/world_1.yaml" )
	debug_arg = DeclareLaunchArgument( 'debug', default_value=TextSubstitution(text='false'), description="If true starts the node in a gdb server on 'localhost:2159'." )
	test_arg = DeclareLaunchArgument( 'test', default_value=TextSubstitution(text='false'), description="If true the root node has a wait in the activation loop that will slow the server down. For the unit tests specifically they rely on this wait to achieve consistent outputs when dynamic changes are given to the server.")

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
			test_arg, 
			debug_group, 
			no_debug_group
		])