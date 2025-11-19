#!/usr/bin/env python3
"""
Analyze the exact timing of voltage commands and changes
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
print("VOLTAGE COMMAND VS ACTUAL VOLTAGE TIMELINE")
print("="*100)

# Track URScript commands
if '/urscript_interface/script_command' in topics:
    string_msg_type = get_message('std_msgs/msg/String')
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/urscript_interface/script_command']}
        ORDER BY timestamp
    """)

    print("\nURSCRIPT COMMANDS SENT:")
    print("-"*50)
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, string_msg_type)
        rel_time = (timestamp - start_time) / 1e9
        if 'set_tool_voltage' in msg.data.lower():
            print(f"{rel_time:7.2f}s: {msg.data}")

# Track actual voltage readings around those times
if '/io_and_status_controller/tool_data' in topics:
    tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')

    print("\nVOLTAGE READINGS (every 2 seconds):")
    print("-"*50)

    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
        ORDER BY timestamp
    """)

    all_messages = cursor.fetchall()

    # Sample every 2 seconds approximately
    sample_interval = 2.0  # seconds
    last_sample_time = -1.0
    prev_voltage = None

    for timestamp, data in all_messages:
        msg = deserialize_message(data, tool_data_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        # Sample at intervals or when voltage changes
        should_print = False
        if rel_time - last_sample_time >= sample_interval:
            should_print = True
            last_sample_time = rel_time
        elif prev_voltage is not None and abs(msg.tool_output_voltage - prev_voltage) > 0.5:
            should_print = True

        if should_print:
            print(f"{rel_time:7.2f}s: tool_output={msg.tool_output_voltage:5.1f}V")

        prev_voltage = msg.tool_output_voltage

# Check for launch-related messages
if '/rosout' in topics:
    log_msg_type = get_message('rcl_interfaces/msg/Log')

    print("\nKEY INITIALIZATION EVENTS:")
    print("-"*50)

    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/rosout']}
        ORDER BY timestamp
    """)

    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, log_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        # Check for launch file timer action messages
        if 'timer' in msg.msg.lower() or 'set_tool_voltage' in msg.msg.lower():
            print(f"{rel_time:7.2f}s: [{msg.name}] {msg.msg[:100]}")

        # Check for external_control program loading
        if 'external_control' in msg.msg.lower() or 'program' in msg.msg.lower():
            if 'sent' in msg.msg.lower() or 'requested' in msg.msg.lower():
                print(f"{rel_time:7.2f}s: [{msg.name}] {msg.msg[:100]}")

print("\n" + "="*100)
print("ANALYSIS:")
print("-"*50)
print("1. Check if set_tool_voltage(0) command was sent by launch file TimerAction")
print("2. Check if voltage actually changed after the command")
print("3. Check timing between program load and voltage command")
print("="*100)

conn.close()