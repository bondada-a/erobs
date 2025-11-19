#!/usr/bin/env python3
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test7"
db_path = f"{bag_path}/tool_voltage_issue_test7_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get topics
cursor.execute("SELECT id, name FROM topics")
topics = {name: topic_id for topic_id, name in cursor.fetchall()}

# Get start time
cursor.execute("SELECT MIN(timestamp) FROM messages")
start_time = cursor.fetchone()[0]

print("="*100)
print("TEST 7 - COMPLETE VOLTAGE ANALYSIS")
print("="*100)

events = []

# Track voltage at key moments
if '/io_and_status_controller/tool_data' in topics:
    tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
        ORDER BY timestamp
    """)

    messages = cursor.fetchall()

    # Sample key moments
    indices = [0, len(messages)//4, len(messages)//2, 3*len(messages)//4, len(messages)-1]

    print("\nVOLTAGE SAMPLES:")
    print("-"*50)
    for idx in indices:
        timestamp, data = messages[idx]
        msg = deserialize_message(data, tool_data_msg_type)
        rel_time = (timestamp - start_time) / 1e9
        print(f"{rel_time:7.2f}s: tool_voltage_48v={msg.tool_voltage_48v:5.1f}V, tool_output_voltage={msg.tool_output_voltage:5.1f}V")

    # Track voltage changes
    prev_48v = None
    prev_output = None
    for timestamp, data in messages:
        msg = deserialize_message(data, tool_data_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        if prev_48v is not None and abs(msg.tool_voltage_48v - prev_48v) > 0.5:
            events.append((rel_time, '⚡ VOLTAGE_48V', f'{prev_48v}V → {msg.tool_voltage_48v}V'))
        if prev_output is not None and abs(msg.tool_output_voltage - prev_output) > 0.5:
            events.append((rel_time, '⚡ OUTPUT_VOLTAGE', f'{prev_output}V → {msg.tool_output_voltage}V'))

        prev_48v = msg.tool_voltage_48v
        prev_output = msg.tool_output_voltage

# Check for voltage-related log messages
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

        # Track all orchestrator voltage messages
        if 'orchestrator' in msg.name.lower() and 'voltage' in msg.msg.lower():
            level = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'L{msg.level}')
            events.append((rel_time, f'📋 {level}', msg.msg[:90]))

        # Track initialize_moveit_stack calls (shows gripper switches)
        if 'MoveIt' in msg.msg or 'gripper' in msg.msg.lower():
            if msg.level >= 20:  # INFO and above
                events.append((rel_time, '🔧 CONFIG', msg.msg[:90]))

        # Track errors
        if msg.level >= 40 and 'zivid' not in msg.name.lower():
            events.append((rel_time, '🚨 ERROR', f'{msg.name}: {msg.msg[:80]}'))

# Check robot mode and safety
if '/io_and_status_controller/safety_mode' in topics:
    safety_msg_type = get_message('ur_dashboard_msgs/msg/SafetyMode')
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/safety_mode']}
        ORDER BY timestamp
    """)
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, safety_msg_type)
        rel_time = (timestamp - start_time) / 1e9
        if msg.mode >= 3:  # Safety issues
            safety_names = {3: 'PROTECTIVE_STOP', 4: 'RECOVERY', 5: 'SAFEGUARD_STOP',
                          6: 'SYSTEM_EMERGENCY_STOP', 7: 'ROBOT_EMERGENCY_STOP',
                          8: 'VIOLATION', 9: 'FAULT'}
            safety_name = safety_names.get(msg.mode, f'MODE_{msg.mode}')
            events.append((rel_time, '🚨 SAFETY', f'*** {safety_name} ***'))

# Check program running
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
        if prev_running is not None and msg.data != prev_running:
            status = "RUNNING ✓" if msg.data else "STOPPED ✗"
            events.append((rel_time, 'PROGRAM', status))
        prev_running = msg.data

# Sort all events
events.sort(key=lambda x: x[0])

# Print timeline
print("\n" + "="*100)
print("TIMELINE OF EVENTS")
print("="*100)
print(f"{'TIME(s)':<10} {'EVENT':<25} {'DETAILS'}")
print("-"*100)

for time, event_type, details in events:
    print(f"{time:<10.2f} {event_type:<25} {details}")

# Summary
print("\n" + "="*100)
print("KEY FINDINGS:")
print("-"*50)

# Check if voltage was set to 0 for standalone
standalone_voltage_set = False
for time, event_type, details in events:
    if 'none gripper' in details.lower() and 'voltage' in details.lower():
        print(f"✓ Attempted to set voltage for standalone at {time:.2f}s: {details}")
        standalone_voltage_set = True
    if 'OUTPUT_VOLTAGE' in event_type and '24V → 0V' in details:
        print(f"✓ Voltage successfully dropped to 0V at {time:.2f}s")
    if 'OUTPUT_VOLTAGE' in event_type and '0V → ' in details and '24V' in details:
        print(f"✗ Voltage increased from 0V at {time:.2f}s: {details}")
    if 'FAULT' in str(details):
        print(f"🚨 FAULT occurred at {time:.2f}s")

if not standalone_voltage_set:
    print("✗ NO ATTEMPT TO SET VOLTAGE FOR STANDALONE DETECTED!")

conn.close()