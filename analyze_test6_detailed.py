#!/usr/bin/env python3
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test6"
db_path = f"{bag_path}/tool_voltage_issue_test6_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get topics
cursor.execute("SELECT id, name FROM topics")
topics = {name: topic_id for topic_id, name in cursor.fetchall()}

# Get start time
cursor.execute("SELECT MIN(timestamp) FROM messages")
start_time = cursor.fetchone()[0]

print("="*100)
print("DETAILED VOLTAGE AND EVENT ANALYSIS - TEST 6")
print("="*100)

events = []

# Track tool voltage changes
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

        # Track significant voltage changes (>0.5V threshold)
        if prev_48v is not None and abs(msg.tool_voltage_48v - prev_48v) > 0.5:
            events.append((rel_time, '⚡ VOLTAGE_48V', f'{prev_48v}V → {msg.tool_voltage_48v}V'))
        if prev_output is not None and abs(msg.tool_output_voltage - prev_output) > 0.5:
            events.append((rel_time, '⚡ OUTPUT_VOLTAGE', f'{prev_output}V → {msg.tool_output_voltage}V'))

        prev_48v = msg.tool_voltage_48v
        prev_output = msg.tool_output_voltage

# Check safety mode for faults
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
            safety_names = {1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP',
                          4: 'RECOVERY', 5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP',
                          7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT'}
            safety_name = safety_names.get(msg.mode, f'UNKNOWN({msg.mode})')
            if msg.mode >= 3:  # Any safety issue
                events.append((rel_time, '🚨 SAFETY_MODE', f'*** {safety_name} ***'))
            else:
                events.append((rel_time, 'SAFETY_MODE', safety_name))
            prev_safety = msg.mode

# Check robot mode
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

# Check program running status
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
        if prev_running is None or msg.data != prev_running:
            status = "RUNNING ✓" if msg.data else "STOPPED ✗"
            events.append((rel_time, 'PROGRAM', status))
            prev_running = msg.data

# Check error/warning logs
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

        # Show orchestrator messages (INFO and above) to track our voltage commands
        if 'orchestrator' in msg.name.lower():
            if 'voltage' in msg.msg.lower() or 'restart' in msg.msg.lower():
                level_name = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'L{msg.level}')
                events.append((rel_time, f'📋 {level_name}', msg.msg[:90]))

        # Show errors from all sources
        if msg.level >= 40 and 'zivid' not in msg.name.lower():
            level_name = {40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'LEVEL_{msg.level}')
            events.append((rel_time, f'🚨 {level_name}', f'{msg.name}: {msg.msg[:80]}'))

# Sort by time
events.sort(key=lambda x: x[0])

# Print timeline
print("\n{:<10} {:<25} {}".format("TIME(s)", "EVENT", "DETAILS"))
print("-"*100)

for time, event_type, details in events:
    print("{:<10.2f} {:<25} {}".format(time, event_type, details))

print("\n" + "="*100)

# Summary: Check initial and final voltage states
cursor.execute(f"""
    SELECT data FROM messages
    WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
    ORDER BY timestamp LIMIT 1
""")
first_tool_data = cursor.fetchone()
if first_tool_data:
    msg = deserialize_message(first_tool_data[0], tool_data_msg_type)
    print(f"\nINITIAL STATE: tool_voltage_48v={msg.tool_voltage_48v}V, tool_output_voltage={msg.tool_output_voltage}V")

conn.close()
