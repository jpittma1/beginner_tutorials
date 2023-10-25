from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    msg_arg = DeclareLaunchArgument('message', default_value = TextSubstitution(text="Nah, Box Car Racer is better!"))
    freq_arg = DeclareLaunchArgument('message_freq', default_value = TextSubstitution(text="1000"))
    ros_bag_arg = DeclareLaunchArgument('rosbag_record', default_value = TextSubstitution(text = "True"), choices = ['True', 'False'], description = "Bool for switching ros bag recording on/off")
    
    publisher = Node(
            package='beginner_tutorials',
            executable='talker',
            parameters=[
                {"message" : LaunchConfiguration('message')},
                {"message_freq" : LaunchConfiguration('message_freq')}
            ]
    )
    
    subscriber = Node(
            package='beginner_tutorials',
            executable='listener'
    )
    
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rosbag_record')),
            cmd=['ros2', 'bag', 'record', '-a'],
        shell=True
    )
    
    return LaunchDescription([
        msg_arg,
        freq_arg,
        ros_bag_arg,
        publisher,
        subscriber,
        recorder
        
    ])