#!/usr/bin/env python3
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test3"
db_path = f"{bag_path}/tool_voltage_issue_test3_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get topics
cursor.execute("SELECT id, name FROM topics")
topics = {name: topic_id for topic_id, name in cursor.fetchall()}

# Get start time
cursor.execute("SELECT MIN(timestamp) FROM messages")
start_time = cursor.fetchone()[0]

print("="*100)
print("TOOL VOLTAGE ISSUE TEST 3 - TIMELINE ANALYSIS")
print("="*100)

events = []

# Check robot mode changes
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
            status = "RUNNING" if msg.data else "STOPPED"
            events.append((rel_time, '⚠️  PROGRAM_STATUS', f'*** {status} ***'))
            prev_running = msg.data

# Check error logs
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
        if msg.level >= 40 and 'zivid' not in msg.name.lower():  # ERROR and above, skip zivid errors
            level_name = {40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'LEVEL_{msg.level}')
            events.append((rel_time, f'⚠️  {level_name}', f'{msg.name}: {msg.msg[:80]}'))

# Sort by time
events.sort(key=lambda x: x[0])

# Print timeline
print("\n{:<10} {:<30} {}".format("TIME(s)", "EVENT", "DETAILS"))
print("-"*100)

for time, event_type, details in events:
    print("{:<10.2f} {:<30} {}".format(time, event_type, details))

print("\n" + "="*100)

# Check if there are any service calls or action topics
print("\nLooking for dashboard service calls...")
cursor.execute("SELECT name FROM topics WHERE name LIKE '%dashboard%'")
dashboard_topics = cursor.fetchall()
if dashboard_topics:
    print("Dashboard topics found:", dashboard_topics)
else:
    print("No dashboard topics in recording")

conn.close()
