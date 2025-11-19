#!/usr/bin/env python3
"""
Detailed timeline analysis to correlate events with voltage states
"""

import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_timeline(bag_path):
    db_path = f"{bag_path}/tool_voltage_issue_0.db3"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get all topic IDs we care about
    cursor.execute("SELECT id, name FROM topics")
    topics = {name: topic_id for topic_id, name in cursor.fetchall()}

    # Get start time
    cursor.execute("SELECT MIN(timestamp) FROM messages")
    start_time = cursor.fetchone()[0]

    print("="*100)
    print("DETAILED TIMELINE ANALYSIS")
    print("="*100)

    # Create unified timeline
    events = []

    # Get tool data (for voltage monitoring)
    if '/io_and_status_controller/tool_data' in topics:
        tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
            ORDER BY timestamp
        """)

        prev_48v = None
        prev_output = None
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, tool_data_msg_type)
            rel_time = (timestamp - start_time) / 1e9

            # Track voltage changes
            if prev_48v is not None and abs(msg.tool_voltage_48v - prev_48v) > 0.5:
                events.append((rel_time, 'VOLTAGE_48V_CHANGE', f'{prev_48v}V -> {msg.tool_voltage_48v}V'))
            if prev_output is not None and abs(msg.tool_output_voltage - prev_output) > 0.5:
                events.append((rel_time, 'OUTPUT_VOLTAGE_CHANGE', f'{prev_output}V -> {msg.tool_output_voltage}V'))

            prev_48v = msg.tool_voltage_48v
            prev_output = msg.tool_output_voltage

    # Get URScript commands
    if '/urscript_interface/script_command' in topics:
        string_msg_type = get_message('std_msgs/msg/String')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/urscript_interface/script_command']}
            ORDER BY timestamp
        """)
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, string_msg_type)
            rel_time = (timestamp - start_time) / 1e9
            events.append((rel_time, 'URSCRIPT_COMMAND', msg.data))

    # Get trajectory controller state changes (indicates motion)
    if '/scaled_joint_trajectory_controller/state' in topics:
        traj_msg_type = get_message('control_msgs/msg/JointTrajectoryControllerState')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/scaled_joint_trajectory_controller/state']}
            ORDER BY timestamp
            LIMIT 1
        """)
        result = cursor.fetchone()
        if result:
            timestamp, _ = result
            rel_time = (timestamp - start_time) / 1e9
            events.append((rel_time, 'TRAJECTORY_START', 'First trajectory point'))

    # Get robot mode changes
    if '/io_and_status_controller/robot_mode' in topics:
        mode_msg_type = get_message('ur_dashboard_msgs/msg/RobotMode')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/io_and_status_controller/robot_mode']}
            ORDER BY timestamp
        """)
        prev_mode = None
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, mode_msg_type)
            rel_time = (timestamp - start_time) / 1e9
            if prev_mode is None or msg.mode != prev_mode:
                mode_names = {0: 'NO_CONTROLLER', 1: 'DISCONNECTED', 2: 'CONFIRM_SAFETY',
                              3: 'BOOTING', 4: 'POWER_OFF', 5: 'POWER_ON', 6: 'IDLE',
                              7: 'BACKDRIVE', 8: 'RUNNING'}
                mode_name = mode_names.get(msg.mode, f'UNKNOWN({msg.mode})')
                events.append((rel_time, 'ROBOT_MODE', mode_name))
                prev_mode = msg.mode

    # Get safety mode changes
    if '/io_and_status_controller/safety_mode' in topics:
        safety_msg_type = get_message('ur_dashboard_msgs/msg/SafetyMode')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/io_and_status_controller/safety_mode']}
            ORDER BY timestamp
        """)
        prev_safety = None
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, safety_msg_type)
            rel_time = (timestamp - start_time) / 1e9
            if prev_safety is None or msg.mode != prev_safety:
                safety_names = {1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP',  4: 'RECOVERY', 5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP',
                               7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT'}
                safety_name = safety_names.get(msg.mode, f'UNKNOWN({msg.mode})')
                if safety_name in ['PROTECTIVE_STOP', 'FAULT', 'VIOLATION']:
                    events.append((rel_time, '⚠️  SAFETY_MODE', f'*** {safety_name} ***'))
                else:
                    events.append((rel_time, 'SAFETY_MODE', safety_name))
                prev_safety = msg.mode

    # Get log messages that might indicate errors
    if '/rosout' in topics:
        log_msg_type = get_message('rcl_interfaces/msg/Log')
        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topics['/rosout']}
            ORDER BY timestamp
        """)
        for timestamp, data in cursor.fetchall():
            msg = deserialize_message(data, log_msg_type)
            rel_time = (timestamp - start_time) / 1e9
            # Only show ERROR and FATAL messages
            if msg.level >= 40:  # ERROR=40, FATAL=50
                level_name = 'ERROR' if msg.level == 40 else 'FATAL' if msg.level == 50 else f'LEVEL_{msg.level}'
                events.append((rel_time, f'⚠️  LOG_{level_name}', f'{msg.name}: {msg.msg}'))

    # Sort all events by time
    events.sort(key=lambda x: x[0])

    # Print timeline
    print("\n{:<10} {:<30} {}".format("TIME(s)", "EVENT", "DETAILS"))
    print("-"*100)

    for time, event_type, details in events:
        print("{:<10.2f} {:<30} {}".format(time, event_type, details))

    conn.close()

    print("\n" + "="*100)
    print("ANALYSIS COMPLETE")
    print("="*100)

if __name__ == "__main__":
    bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue"
    analyze_timeline(bag_path)
