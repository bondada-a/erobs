#!/usr/bin/env python3
"""
Unified ROS 2 Bag Analyzer for Debugging UR Robot Issues

This tool consolidates multiple analysis scripts into a single, flexible analyzer
for ROS 2 bag files containing UR robot data.

Usage:
    ./analyze_bag.py <bag_path> [options]

Examples:
    # Basic voltage analysis
    ./analyze_bag.py recorded_data/my_bag

    # Focus on specific time window
    ./analyze_bag.py recorded_data/my_bag --window 55 60

    # Detailed analysis with all robot states
    ./analyze_bag.py recorded_data/my_bag --detailed

    # Focus on voltage changes only
    ./analyze_bag.py recorded_data/my_bag --focus voltage
"""

import argparse
import sqlite3
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class BagAnalyzer:
    """Analyzes ROS 2 bag files for UR robot debugging"""

    # ANSI color codes for terminal output
    COLORS = {
        'RED': '\033[1;31m',
        'YELLOW': '\033[1;33m',
        'GREEN': '\033[1;32m',
        'CYAN': '\033[1;36m',
        'BRIGHT_RED': '\033[1;91m',
        'RESET': '\033[0m'
    }

    def __init__(self, bag_path, voltage_threshold=0.5, detailed=False,
                 time_window=None, focus=None, no_color=False):
        """
        Initialize the bag analyzer

        Args:
            bag_path: Path to the bag directory
            voltage_threshold: Voltage change threshold to detect (default 0.5V)
            detailed: Show detailed robot states and safety info
            time_window: Tuple of (start_time, end_time) to focus analysis
            focus: Specific focus area ('voltage', 'safety', 'logs', 'all')
            no_color: Disable colored output
        """
        self.bag_path = Path(bag_path)
        self.voltage_threshold = voltage_threshold
        self.detailed = detailed
        self.time_window = time_window
        self.focus = focus or 'all'
        self.no_color = no_color

        # Find the database file
        db_files = list(self.bag_path.glob("*.db3"))
        if not db_files:
            raise FileNotFoundError(f"No .db3 file found in {bag_path}")

        self.db_path = db_files[0]
        self.conn = sqlite3.connect(str(self.db_path))
        self.cursor = self.conn.cursor()

        # Get topic mapping
        self.cursor.execute("SELECT id, name FROM topics")
        self.topics = {name: topic_id for topic_id, name in self.cursor.fetchall()}

        # Get start time
        self.cursor.execute("SELECT MIN(timestamp) FROM messages")
        self.start_time = self.cursor.fetchone()[0]

        # Storage for timeline events
        self.events = []

    def colorize(self, text, color_name):
        """Apply color to text if colors are enabled"""
        if self.no_color:
            return text
        return f"{self.COLORS[color_name]}{text}{self.COLORS['RESET']}"

    def in_time_window(self, rel_time):
        """Check if time is within the specified window"""
        if self.time_window is None:
            return True
        start, end = self.time_window
        return start <= rel_time <= end

    def should_analyze(self, category):
        """Check if we should analyze this category based on focus"""
        return self.focus == 'all' or self.focus == category

    def analyze_voltage(self):
        """Analyze tool voltage changes"""
        if not self.should_analyze('voltage'):
            return

        topic_name = '/io_and_status_controller/tool_data'
        if topic_name not in self.topics:
            print(f"⚠ Topic {topic_name} not found in bag")
            return

        tool_data_msg_type = get_message('ur_msgs/msg/ToolDataMsg')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        messages = self.cursor.fetchall()

        # Record initial state
        if messages:
            timestamp, data = messages[0]
            msg = deserialize_message(data, tool_data_msg_type)
            self.events.append((
                0.0,
                'INITIAL_VOLTAGE',
                f'tool_output={msg.tool_output_voltage}V, tool_48v={msg.tool_voltage_48v}V'
            ))

        # Track voltage changes
        prev_output = None
        prev_48v = None

        for timestamp, data in messages:
            msg = deserialize_message(data, tool_data_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            # Detect significant voltage changes
            if prev_output is not None and abs(msg.tool_output_voltage - prev_output) > self.voltage_threshold:
                self.events.append((
                    rel_time,
                    '⚡ VOLTAGE_OUTPUT',
                    f'{prev_output}V → {msg.tool_output_voltage}V'
                ))

            if prev_48v is not None and abs(msg.tool_voltage_48v - prev_48v) > self.voltage_threshold:
                self.events.append((
                    rel_time,
                    '⚡ VOLTAGE_48V',
                    f'{prev_48v}V → {msg.tool_voltage_48v}V'
                ))

            prev_output = msg.tool_output_voltage
            prev_48v = msg.tool_voltage_48v

        # Record final state
        if messages:
            self.events.append((
                rel_time,
                'FINAL_VOLTAGE',
                f'tool_output={prev_output}V, tool_48v={prev_48v}V'
            ))

    def analyze_robot_mode(self):
        """Analyze robot mode changes"""
        if not self.detailed:
            return

        topic_name = '/io_and_status_controller/robot_mode'
        if topic_name not in self.topics:
            return

        mode_names = {
            0: 'NO_CONTROLLER', 1: 'DISCONNECTED', 2: 'CONFIRM_SAFETY',
            3: 'BOOTING', 4: 'POWER_OFF', 5: 'POWER_ON', 6: 'IDLE',
            7: 'BACKDRIVE', 8: 'RUNNING'
        }

        mode_msg_type = get_message('ur_dashboard_msgs/msg/RobotMode')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        prev_mode = None
        for timestamp, data in self.cursor.fetchall():
            msg = deserialize_message(data, mode_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            if prev_mode is None or msg.mode != prev_mode:
                mode_name = mode_names.get(msg.mode, f'UNKNOWN({msg.mode})')
                self.events.append((rel_time, '🤖 ROBOT_MODE', mode_name))
                prev_mode = msg.mode

    def analyze_safety_mode(self):
        """Analyze safety mode changes"""
        if not (self.detailed or self.should_analyze('safety')):
            return

        topic_name = '/io_and_status_controller/safety_mode'
        if topic_name not in self.topics:
            return

        safety_names = {
            1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP',
            4: 'RECOVERY', 5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP',
            7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT'
        }

        safety_msg_type = get_message('ur_dashboard_msgs/msg/SafetyMode')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        prev_safety = None
        for timestamp, data in self.cursor.fetchall():
            msg = deserialize_message(data, safety_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            if prev_safety is None or msg.mode != prev_safety:
                safety_name = safety_names.get(msg.mode, f'UNKNOWN({msg.mode})')
                if msg.mode >= 3:  # Safety issue
                    self.events.append((rel_time, '🚨 SAFETY', f'*** {safety_name} ***'))
                else:
                    self.events.append((rel_time, 'SAFETY', safety_name))
                prev_safety = msg.mode

    def analyze_program_status(self):
        """Analyze robot program running status"""
        topic_name = '/io_and_status_controller/robot_program_running'
        if topic_name not in self.topics:
            return

        bool_msg_type = get_message('std_msgs/msg/Bool')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        prev_running = None
        for timestamp, data in self.cursor.fetchall():
            msg = deserialize_message(data, bool_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            if prev_running is None or msg.data != prev_running:
                status = "RUNNING ✓" if msg.data else "STOPPED ✗"
                self.events.append((rel_time, '🔄 PROGRAM', status))
                prev_running = msg.data

    def analyze_urscript_commands(self):
        """Analyze URScript commands sent to the robot"""
        if not self.should_analyze('voltage') and not self.detailed:
            return

        topic_name = '/urscript_interface/script_command'
        if topic_name not in self.topics:
            return

        string_msg_type = get_message('std_msgs/msg/String')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        for timestamp, data in self.cursor.fetchall():
            msg = deserialize_message(data, string_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            # Truncate long commands
            cmd = msg.data[:100]
            self.events.append((rel_time, '📝 URSCRIPT', cmd))

    def analyze_logs(self):
        """Analyze ROS log messages"""
        if not self.should_analyze('logs') and not self.detailed:
            return

        topic_name = '/rosout'
        if topic_name not in self.topics:
            return

        log_msg_type = get_message('rcl_interfaces/msg/Log')
        self.cursor.execute(f"""
            SELECT timestamp, data
            FROM messages
            WHERE topic_id = {self.topics[topic_name]}
            ORDER BY timestamp
        """)

        level_names = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

        for timestamp, data in self.cursor.fetchall():
            msg = deserialize_message(data, log_msg_type)
            rel_time = (timestamp - self.start_time) / 1e9

            if not self.in_time_window(rel_time):
                continue

            # Skip noisy messages
            if 'zivid' in msg.name.lower():
                continue

            # Capture voltage-related messages
            if 'voltage' in msg.msg.lower():
                level = level_names.get(msg.level, f'L{msg.level}')
                message = msg.msg[:90]
                self.events.append((rel_time, f'📋 {level}', message))

            # Capture errors (INFO and above)
            elif msg.level >= 40:
                level = level_names.get(msg.level, f'L{msg.level}')
                message = f'{msg.name}: {msg.msg[:80]}'
                self.events.append((rel_time, f'🚨 {level}', message))

            # Capture MoveIt/gripper configuration changes
            elif self.detailed and any(kw in msg.msg.lower() for kw in ['moveit', 'gripper', 'configuration']):
                if msg.level >= 20:
                    message = msg.msg[:90]
                    self.events.append((rel_time, '🔧 CONFIG', message))

    def print_header(self):
        """Print analysis header"""
        print("=" * 100)
        print(f"ROS 2 BAG ANALYSIS: {self.bag_path.name}")
        print("=" * 100)
        print(f"Database: {self.db_path.name}")
        print(f"Topics found: {len(self.topics)}")

        if self.time_window:
            print(f"Time window: {self.time_window[0]:.2f}s - {self.time_window[1]:.2f}s")
        if self.focus != 'all':
            print(f"Focus: {self.focus}")

        print()

    def print_timeline(self):
        """Print the timeline of events"""
        if not self.events:
            print("No events found in the specified criteria.")
            return

        # Sort events by time
        self.events.sort(key=lambda x: x[0])

        print("=" * 100)
        print("TIMELINE OF EVENTS")
        print("=" * 100)
        print(f"{'TIME(s)':<10} {'EVENT':<25} {'DETAILS'}")
        print("-" * 100)

        for time, event_type, details in self.events:
            # Apply color highlighting
            if 'set_tool_voltage' in details.lower():
                line = self.colorize(f"{time:<10.2f} {event_type:<25} {details}", 'YELLOW')
            elif 'VOLTAGE' in event_type and '⚡' in event_type:
                line = self.colorize(f"{time:<10.2f} {event_type:<25} {details}", 'RED')
            elif 'Verifying' in details or 'verified' in details.lower():
                line = self.colorize(f"{time:<10.2f} {event_type:<25} {details}", 'CYAN')
            elif 'ERROR' in event_type or 'FAULT' in str(details):
                line = self.colorize(f"{time:<10.2f} {event_type:<25} {details}", 'BRIGHT_RED')
            else:
                line = f"{time:<10.2f} {event_type:<25} {details}"

            print(line)

    def print_summary(self):
        """Print analysis summary"""
        print("\n" + "=" * 100)
        print("KEY FINDINGS:")
        print("-" * 50)

        # Check for voltage setting commands
        voltage_commands = [e for e in self.events if 'set_tool_voltage' in str(e[2]).lower()]
        if voltage_commands:
            print(f"✓ Found {len(voltage_commands)} voltage set commands")
            for time, _, details in voltage_commands:
                print(f"  - {time:.2f}s: {details[:80]}")

        # Check for voltage changes
        voltage_changes = [e for e in self.events if '⚡ VOLTAGE' in e[1]]
        if voltage_changes:
            print(f"\n✓ Detected {len(voltage_changes)} significant voltage changes")
            for time, event_type, details in voltage_changes:
                print(f"  - {time:.2f}s: {details}")
        else:
            print("\n✗ No significant voltage changes detected")

        # Check for safety issues
        safety_issues = [e for e in self.events if '🚨 SAFETY' in e[1]]
        if safety_issues:
            print(f"\n⚠ Found {len(safety_issues)} safety events")
            for time, _, details in safety_issues:
                print(f"  - {time:.2f}s: {details}")

        # Check for errors
        errors = [e for e in self.events if 'ERROR' in e[1] or 'FATAL' in e[1]]
        if errors:
            print(f"\n⚠ Found {len(errors)} error messages")
            for time, event_type, details in errors[:5]:  # Show first 5
                print(f"  - {time:.2f}s [{event_type}]: {details[:70]}")
            if len(errors) > 5:
                print(f"  ... and {len(errors) - 5} more")

        print("=" * 100)

    def analyze(self):
        """Run complete analysis"""
        self.print_header()

        # Run all analysis modules
        self.analyze_voltage()
        self.analyze_robot_mode()
        self.analyze_safety_mode()
        self.analyze_program_status()
        self.analyze_urscript_commands()
        self.analyze_logs()

        # Print results
        self.print_timeline()
        self.print_summary()

    def close(self):
        """Clean up database connection"""
        self.conn.close()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze ROS 2 bag files for UR robot debugging",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s recorded_data/my_bag
  %(prog)s recorded_data/my_bag --window 55 60
  %(prog)s recorded_data/my_bag --detailed
  %(prog)s recorded_data/my_bag --focus voltage --threshold 0.2
        """
    )

    parser.add_argument('bag_path', help='Path to the ROS 2 bag directory')
    parser.add_argument('--detailed', '-d', action='store_true',
                        help='Show detailed robot states and configuration changes')
    parser.add_argument('--window', '-w', nargs=2, type=float, metavar=('START', 'END'),
                        help='Focus on specific time window (in seconds)')
    parser.add_argument('--focus', '-f', choices=['voltage', 'safety', 'logs', 'all'],
                        default='all', help='Focus analysis on specific area')
    parser.add_argument('--threshold', '-t', type=float, default=0.5,
                        help='Voltage change threshold in volts (default: 0.5)')
    parser.add_argument('--no-color', action='store_true',
                        help='Disable colored output')

    args = parser.parse_args()

    # Validate bag path
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag path '{bag_path}' does not exist")
        sys.exit(1)

    # Create and run analyzer
    try:
        analyzer = BagAnalyzer(
            bag_path=bag_path,
            voltage_threshold=args.threshold,
            detailed=args.detailed,
            time_window=tuple(args.window) if args.window else None,
            focus=args.focus,
            no_color=args.no_color
        )
        analyzer.analyze()
        analyzer.close()
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
