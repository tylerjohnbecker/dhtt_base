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
                                	Node( package='dhtt_cooking', executable='dhtt_cooking', output='log', prefix=[debug_text], emulate_tty=True ),
								]
							)

	group_no_test = GroupAction(
								condition = LaunchConfigurationEquals('test', 'false'),
								actions= [
									Node( package='dhtt', executable='start_server', output='log', prefix=[debug_text], emulate_tty=True, arguments=[] ),
                                	Node( package='dhtt_cooking', executable='dhtt_cooking', output='log', prefix=[debug_text], emulate_tty=True ),
								]
							)

	return [group_test, group_no_test]

def generate_launch_description():

	### ARGUMENTS
	params_file_arg = DeclareLaunchArgument( 'params_file' , default_value=TextSubstitution(text='') )
	debug_arg = DeclareLaunchArgument( 'debug', default_value=TextSubstitution(text='false') )
	test_arg = DeclareLaunchArgument( 'test', default_value=TextSubstitution(text='false'))

	debug_group = GroupAction(
							condition= LaunchConfigurationEquals('debug', 'true'),
							actions=get_main_node('gdbserver localhost:3111')
						  )

	no_debug_group = GroupAction(
									condition= LaunchConfigurationEquals('debug', 'false'),
									actions=get_main_node()
								)

	return LaunchDescription([ 
			params_file_arg, 
			debug_arg, 
			test_arg,
			debug_group, 
			no_debug_group
		])