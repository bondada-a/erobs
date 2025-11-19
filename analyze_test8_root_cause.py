#!/usr/bin/env python3
"""
Deep analysis of test8 to find why voltage stays at 24V after standalone initialization
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
print("TEST 8 - WHY VOLTAGE STAYS AT 24V ANALYSIS")
print("="*100)

# Track all voltage-related events in chronological order
events = []

# Get tool voltage data
if '/io_and_status_controller/tool_data' in topics:
    tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')

    # Get first and last readings
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
        ORDER BY timestamp
        LIMIT 1
    """)
    first_msg = cursor.fetchone()
    if first_msg:
        timestamp, data = first_msg
        msg = deserialize_message(data, tool_data_msg_type)
        events.append((0.0, 'INITIAL_VOLTAGE', f'tool_output={msg.tool_output_voltage}V, tool_48v={msg.tool_voltage_48v}V'))

    # Track voltage changes
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = {topics['/io_and_status_controller/tool_data']}
        ORDER BY timestamp
    """)

    prev_output = None
    prev_48v = None
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, tool_data_msg_type)
        rel_time = (timestamp - start_time) / 1e9

        # Detect changes
        if prev_output is not None and abs(msg.tool_output_voltage - prev_output) > 0.5:
            events.append((rel_time, '⚡ VOLTAGE_CHANGE', f'output: {prev_output}V → {msg.tool_output_voltage}V'))
        if prev_48v is not None and abs(msg.tool_voltage_48v - prev_48v) > 0.5:
            events.append((rel_time, '⚡ VOLTAGE_48V_CHG', f'48v: {prev_48v}V → {msg.tool_voltage_48v}V'))

        prev_output = msg.tool_output_voltage
        prev_48v = msg.tool_voltage_48v

    # Final voltage
    events.append((rel_time, 'FINAL_VOLTAGE', f'output={prev_output}V, 48v={prev_48v}V'))

# Get all log messages related to voltage and MoveIt
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

        # Capture all voltage-related messages
        if 'voltage' in msg.msg.lower():
            level = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'L{msg.level}')
            # Shorten the message for readability
            message = msg.msg.replace('mtc_orchestrator_action_server: ', '')[:90]
            events.append((rel_time, f'LOG_{level}', message))

        # Capture MoveIt/gripper changes
        if any(keyword in msg.msg.lower() for keyword in ['moveit', 'gripper', 'switching', 'configuration']):
            if msg.level >= 20:  # INFO and above
                message = msg.msg.replace('mtc_orchestrator_action_server: ', '')[:90]
                events.append((rel_time, 'CONFIG', message))

        # Capture dashboard play
        if 'dashboard' in msg.msg.lower() or 'play' in msg.msg.lower():
            message = msg.msg.replace('mtc_orchestrator_action_server: ', '')[:90]
            events.append((rel_time, '🎮 DASHBOARD', message))

        # Capture program status
        if 'program' in msg.msg.lower() and 'robot' in msg.msg.lower():
            message = msg.msg.replace('mtc_orchestrator_action_server: ', '')[:90]
            events.append((rel_time, '🤖 PROGRAM', message))

# Check robot program running status
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
            events.append((rel_time, '🔄 PROG_STATUS', status))
            prev_running = msg.data

# Check for URScript commands sent
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
        events.append((rel_time, '📝 URSCRIPT_CMD', msg.data))

# Sort events chronologically
events.sort(key=lambda x: x[0])

# Print timeline
print("\n" + "="*100)
print("CHRONOLOGICAL EVENT TIMELINE")
print("="*100)
print(f"{'TIME(s)':<10} {'EVENT':<20} {'DETAILS'}")
print("-"*100)

for time, event_type, details in events:
    # Highlight important events
    if 'set_tool_voltage' in details.lower():
        print(f"\033[1;33m{time:<10.2f} {event_type:<20} {details}\033[0m")  # Yellow
    elif 'VOLTAGE_CHANGE' in event_type:
        print(f"\033[1;31m{time:<10.2f} {event_type:<20} {details}\033[0m")  # Red
    elif 'Verifying' in details or 'verified' in details.lower():
        print(f"\033[1;36m{time:<10.2f} {event_type:<20} {details}\033[0m")  # Cyan
    elif 'ERROR' in event_type or 'CRITICAL' in details:
        print(f"\033[1;91m{time:<10.2f} {event_type:<20} {details}\033[0m")  # Bright Red
    else:
        print(f"{time:<10.2f} {event_type:<20} {details}")

# Analysis summary
print("\n" + "="*100)
print("KEY FINDINGS:")
print("-"*50)

# Find when voltage was supposed to be set to 0V
set_to_0v_times = []
for time, event_type, details in events:
    if 'set_tool_voltage(0)' in details.lower() or 'setting tool voltage to 0v' in details.lower():
        set_to_0v_times.append(time)
        print(f"✓ Command to set voltage to 0V sent at {time:.2f}s")

# Check if voltage actually changed
voltage_dropped = False
for time, event_type, details in events:
    if 'VOLTAGE_CHANGE' in event_type and '→ 0' in details:
        voltage_dropped = True
        print(f"✓ Voltage dropped to 0V at {time:.2f}s")
        break

if not voltage_dropped:
    print("✗ VOLTAGE NEVER DROPPED TO 0V!")

# Check verification results
for time, event_type, details in events:
    if 'voltage mismatch' in details.lower() or 'voltage verification failed' in details.lower():
        print(f"✓ Verification correctly detected issue at {time:.2f}s: {details}")

print("\n" + "="*100)
print("HYPOTHESIS:")
print("-"*50)

# Analyze the pattern
if set_to_0v_times and not voltage_dropped:
    print("The set_tool_voltage(0) command was sent but the voltage did NOT change.")
    print("Possible causes:")
    print("1. The URScript command is not being executed by the robot")
    print("2. The robot program being loaded overrides the voltage setting")
    print("3. The socket connection to port 30002 is not working correctly")
    print("4. The robot is in a state where it cannot change voltage")

conn.close()