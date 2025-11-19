#!/usr/bin/env python3
"""
Analyze tool voltage from rosbag to debug voltage transition issue
"""

import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import struct

def analyze_bag(bag_path):
    # Connect to the SQLite database
    db_path = f"{bag_path}/tool_voltage_issue_0.db3"
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get topic ID for tool_data
    cursor.execute("SELECT id FROM topics WHERE name='/io_and_status_controller/tool_data'")
    tool_data_topic_id = cursor.fetchone()[0]

    # Get topic ID for io_states
    cursor.execute("SELECT id FROM topics WHERE name='/io_and_status_controller/io_states'")
    io_states_topic_id = cursor.fetchone()[0]

    # Get topic ID for urscript commands
    cursor.execute("SELECT id FROM topics WHERE name='/urscript_interface/script_command'")
    result = cursor.fetchone()
    urscript_topic_id = result[0] if result else None

    print("="*80)
    print("TOOL VOLTAGE ANALYSIS")
    print("="*80)

    # Get message type
    msg_type = get_message('ur_msgs/msg/ToolDataMsg')

    # Query tool_data messages
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {tool_data_topic_id}
        ORDER BY timestamp
    """)

    tool_data_messages = cursor.fetchall()

    print(f"\nTotal tool_data messages: {len(tool_data_messages)}")

    # Sample messages throughout the recording
    indices_to_check = [0, len(tool_data_messages)//4, len(tool_data_messages)//2,
                        3*len(tool_data_messages)//4, len(tool_data_messages)-1]

    print("\n" + "="*80)
    print("TOOL VOLTAGE OVER TIME (sampled)")
    print("="*80)

    voltages = []
    for idx in indices_to_check:
        timestamp, data = tool_data_messages[idx]
        msg = deserialize_message(data, msg_type)

        # Convert nanoseconds to seconds
        time_sec = timestamp / 1e9
        relative_time = (timestamp - tool_data_messages[0][0]) / 1e9

        voltages.append((relative_time, msg.tool_voltage_48v))

        print(f"\nTime: {relative_time:.2f}s | Tool Voltage 48V: {msg.tool_voltage_48v}V")
        print(f"  Tool Output Voltage: {msg.tool_output_voltage}V")
        print(f"  Tool Current: {msg.tool_current}A")
        print(f"  Tool Temperature: {msg.tool_temperature}°C")

    # Check for voltage transitions
    print("\n" + "="*80)
    print("VOLTAGE TRANSITION DETECTION")
    print("="*80)

    prev_voltage = None
    transitions = []

    for timestamp, data in tool_data_messages:
        msg = deserialize_message(data, msg_type)
        relative_time = (timestamp - tool_data_messages[0][0]) / 1e9

        if prev_voltage is not None and abs(msg.tool_voltage_48v - prev_voltage) > 1.0:
            transitions.append((relative_time, prev_voltage, msg.tool_voltage_48v))
            print(f"\n⚠️  VOLTAGE TRANSITION at {relative_time:.2f}s: {prev_voltage}V -> {msg.tool_voltage_48v}V")

        prev_voltage = msg.tool_voltage_48v

    if not transitions:
        print("\n⚠️  NO VOLTAGE TRANSITIONS DETECTED!")
        print(f"   Voltage remained constant at ~{voltages[0][1]}V throughout recording")

    # Check URScript commands
    if urscript_topic_id:
        print("\n" + "="*80)
        print("URSCRIPT COMMANDS")
        print("="*80)

        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {urscript_topic_id}
            ORDER BY timestamp
        """)

        urscript_messages = cursor.fetchall()
        string_msg_type = get_message('std_msgs/msg/String')

        for timestamp, data in urscript_messages:
            msg = deserialize_message(data, string_msg_type)
            relative_time = (timestamp - tool_data_messages[0][0]) / 1e9
            print(f"\nTime: {relative_time:.2f}s")
            print(f"  Command: {msg.data}")

    # Check I/O states for digital outputs (tool voltage is often controlled via digital outputs)
    print("\n" + "="*80)
    print("DIGITAL I/O STATES (sampled)")
    print("="*80)

    io_msg_type = get_message('ur_msgs/msg/IOStates')

    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {io_states_topic_id}
        ORDER BY timestamp
    """)

    io_messages = cursor.fetchall()

    for idx in indices_to_check:
        timestamp, data = io_messages[idx]
        msg = deserialize_message(data, io_msg_type)
        relative_time = (timestamp - tool_data_messages[0][0]) / 1e9

        print(f"\nTime: {relative_time:.2f}s")
        print(f"  Digital Outputs: {[f'DO{i}={pin.state}' for i, pin in enumerate(msg.digital_out_states)]}")
        print(f"  Analog Outputs: {[f'AO{i}={pin.state:.2f}' for i, pin in enumerate(msg.analog_out_states)]}")

    conn.close()

    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Recording duration: {(tool_data_messages[-1][0] - tool_data_messages[0][0])/1e9:.2f}s")
    print(f"Voltage transitions detected: {len(transitions)}")
    if transitions:
        for time, v_old, v_new in transitions:
            print(f"  - {time:.2f}s: {v_old}V -> {v_new}V")
    else:
        print("  ⚠️  No voltage transitions - this is likely the problem!")

if __name__ == "__main__":
    bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue"
    analyze_bag(bag_path)
