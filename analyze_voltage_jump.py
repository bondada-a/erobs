#!/usr/bin/env python3
"""
Analyze what happens when voltage jumps back to 24V around 57.88s
"""
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test8"
db_path = f"{bag_path}/tool_voltage_issue_test8_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get topics
cursor.execute("SELECT id, name FROM topics")
topics = {name: topic_id for topic_id, name in cursor.fetchall()}

# Get start time
cursor.execute("SELECT MIN(timestamp) FROM messages")
start_time = cursor.fetchone()[0]

print("="*100)
print("CRITICAL MOMENT ANALYSIS: Voltage Jump at ~57.88s")
print("="*100)

# Target time window: 55-60 seconds
start_window = 55.0
end_window = 60.0

events = []

# Get all log messages in this window
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

        if start_window <= rel_time <= end_window:
            # Skip noisy Zivid messages
            if 'zivid' not in msg.name.lower():
                events.append((rel_time, 'LOG', f"[{msg.name}] {msg.msg[:120]}"))

# Get tool voltage changes
if '/io_and_status_controller/tool_data' in topics:
    tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
        ORDER BY timestamp
    """)

    prev_voltage = None
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, tool_data_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        if start_window <= rel_time <= end_window:
            if prev_voltage is not None and abs(msg.tool_output_voltage - prev_voltage) > 0.5:
                events.append((rel_time, '⚡VOLTAGE', f"Changed: {prev_voltage}V → {msg.tool_output_voltage}V"))
            prev_voltage = msg.tool_output_voltage

# Get robot program status changes
if '/io_and_status_controller/robot_program_running' in topics:
    bool_msg_type = get_message('std_msgs/msg/Bool')
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/robot_program_running']}
        ORDER BY timestamp
    """)

    prev_running = None
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, bool_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        if start_window <= rel_time <= end_window:
            if prev_running is not None and msg.data != prev_running:
                status = "STARTED" if msg.data else "STOPPED"
                events.append((rel_time, '🤖PROGRAM', status))
            prev_running = msg.data

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

        if start_window <= rel_time <= end_window:
            events.append((rel_time, '📝URSCRIPT', msg.data[:100]))

# Sort and display
events.sort(key=lambda x: x[0])

print(f"\nTIMELINE ({start_window}s - {end_window}s):")
print("-"*100)
print(f"{'TIME(s)':<10} {'TYPE':<12} {'DETAILS'}")
print("-"*100)

for time, event_type, details in events:
    if 'VOLTAGE' in event_type:
        print(f"\033[1;31m{time:<10.3f} {event_type:<12} {details}\033[0m")  # Red for voltage
    elif 'PROGRAM' in event_type:
        print(f"\033[1;33m{time:<10.3f} {event_type:<12} {details}\033[0m")  # Yellow for program
    else:
        print(f"{time:<10.3f} {event_type:<12} {details}")

print("\n" + "="*100)
print("HYPOTHESIS:")
print("-"*50)
print("The voltage jump at 57.88s appears to be caused by:")
print("1. The external_control program being restarted")
print("2. A URScript command being sent")
print("3. MoveIt/controller reinitialization")
print("Look for events just before the voltage change!")
print("="*100)

conn.close()