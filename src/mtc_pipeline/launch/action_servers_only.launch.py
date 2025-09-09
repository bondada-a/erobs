#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Modular Action Servers Only (for testing)
    pickplace_action_server = Node(
        package='mtc_pipeline',
        executable='pickplace_action_server',
        name='pickplace_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    toolexchange_action_server = Node(
        package='mtc_pipeline',
        executable='toolexchange_action_server',
        name='toolexchange_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    moveto_action_server = Node(
        package='mtc_pipeline',
        executable='moveto_action_server',
        name='moveto_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    endeffector_action_server = Node(
        package='mtc_pipeline',
        executable='endeffector_action_server',
        name='endeffector_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        pickplace_action_server,
        toolexchange_action_server,
        moveto_action_server,
        endeffector_action_server,
    ])
