#!/usr/bin/env python3
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test"
db_path = f"{bag_path}/tool_voltage_issue_test_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# First check what tables exist
cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = cursor.fetchall()
print("Tables:", tables)

# Find topics table (might have different name)
topic_table = None
for table in tables:
    if 'topic' in table[0].lower():
        topic_table = table[0]
        break

if topic_table:
    print(f"\nUsing table: {topic_table}")
    cursor.execute(f"SELECT * FROM {topic_table} WHERE name LIKE '%rosout%'")
    rosout_info = cursor.fetchone()
    print(f"Rosout topic: {rosout_info}")

    if rosout_info:
        topic_id = rosout_info[0]
        log_msg_type = get_message('rcl_interfaces/msg/Log')

        cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {topic_id}
            ORDER BY timestamp
        """)

        start_time = None
        print("\n" + "="*100)
        print("LOG MESSAGES")
        print("="*100)

        for timestamp, data in cursor.fetchall():
            if start_time is None:
                start_time = timestamp

            msg = deserialize_message(data, log_msg_type)
            rel_time = (timestamp - start_time) / 1e9

            # Show all ERROR and higher messages
            if msg.level >= 40:  # ERROR=40, FATAL=50
                level_name = {40: 'ERROR', 50: 'FATAL'}.get(msg.level, f'LEVEL_{msg.level}')
                print(f"\n[{rel_time:7.2f}s] {level_name} - {msg.name}")
                print(f"  {msg.msg}")

conn.close()
