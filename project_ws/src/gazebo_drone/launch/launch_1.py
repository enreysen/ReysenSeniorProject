import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from launch.actions import ExecuteProcess


def generate_launch_description():
    
    # gazebo configuration
    my_pkg_share = FindPackageShare(package='gazebo_drone').find('gazebo_drone')
    world_path = os.path.join(my_pkg_share, 'worlds', 'challenge_1.world')
    gazebo_ros_pkg =  FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    gazebo_config = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
    		os.path.join(
    		# directory of launch folder, launch, name of launch script
    			gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
   		
   		launch_arguments = {'world': world_path}.items(),
	)
    
    
    return LaunchDescription([

        gazebo_config#,
        #Node(
	    #package='gazebo_drone',
	    #namespace='aruco_tracker',
	    #executable='image_pub_sub',
	    #name='image_pub_sub',
	    #output='screen')
	
    ])

# source: https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/
