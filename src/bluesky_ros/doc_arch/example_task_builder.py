#!/usr/bin/env python3
"""
Example: Using TaskBuilder with Bluesky

Shows the improvement from using named locations instead of file paths
"""

import sys
sys.path.insert(0, 'src')

import rclpy
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_ros.simple_mtc_bluesky import MTCDevice
from bluesky_ros.task_builder import TaskBuilder


# Initialize
rclpy.init()
RE = RunEngine({})
mtc = MTCDevice("robot")
builder = TaskBuilder()

# Robot IP (change to your robot)
ROBOT_IP = '10.69.26.90'


# ============================================================
# BEFORE: Using file paths (old way)
# ============================================================

def old_way_pick_and_place():
    """Old approach: Need separate JSON files for each operation"""

    print("\n" + "="*60)
    print("OLD WAY: Using pre-made JSON files")
    print("="*60)

    # Problem: Need to create/maintain separate JSON files
    yield from bps.abs_set(mtc, {
        'json_file': 'task_sequences/triple_test.json',  # 380 lines!
        'robot_ip': ROBOT_IP
    })


# ============================================================
# AFTER: Using TaskBuilder (new way)
# ============================================================

def new_way_simple_move():
    """New approach: Build tasks from named locations"""

    print("\n" + "="*60)
    print("NEW WAY: Build tasks on the fly")
    print("="*60)

    # Just move to a named location
    json_file = builder.move_to('pickup_approach', planning_type='joint')

    yield from bps.abs_set(mtc, {
        'json_file': json_file,
        'robot_ip': ROBOT_IP
    })

    print("✅ Moved to pickup_approach")


def new_way_pick_sequence():
    """Build a pick sequence using 3 location names"""

    print("\n" + "="*60)
    print("NEW WAY: Pick sequence from 3 locations")
    print("="*60)

    # Build pick sequence from location names
    json_file = builder.pick_sequence(
        approach='pickup_approach',
        grasp='pickup',
        retreat='post_pickup_camera_safety',
        gripper='hande'
    )

    yield from bps.abs_set(mtc, {
        'json_file': json_file,
        'robot_ip': ROBOT_IP
    })

    print("✅ Pick sequence completed")


def new_way_custom_workflow():
    """Build completely custom workflow"""

    print("\n" + "="*60)
    print("NEW WAY: Custom multi-step workflow")
    print("="*60)

    # Step 1: Move to vision position
    json_file = builder.move_to('vision_approach')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
    print("  ✓ At vision position")

    # Step 2: Pick sample
    json_file = builder.pick_sequence('pickup_approach', 'pickup', 'post_pickup_camera_safety')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
    print("  ✓ Picked sample")

    # Step 3: Place sample
    json_file = builder.place_sequence('place_approach', 'place', 'post_pickup_camera_safety')
    yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})
    print("  ✓ Placed sample")

    print("✅ Complete workflow finished")


def new_way_tool_change():
    """Automatic tool changing"""

    print("\n" + "="*60)
    print("NEW WAY: Tool change sequence")
    print("="*60)

    # Change from hande to epick
    json_file = builder.tool_change('hande', 'epick')

    yield from bps.abs_set(mtc, {
        'json_file': json_file,
        'robot_ip': ROBOT_IP
    })

    print("✅ Changed from hande to epick")


def new_way_multi_sample():
    """Process multiple samples - THIS IS THE POWER!"""

    print("\n" + "="*60)
    print("NEW WAY: Multi-sample processing")
    print("="*60)

    # Process 3 samples with vacuum gripper
    sample_locations = [
        ('vacuum_pickup_approach', 'vacuum_pickup', 'vacuum_post_pickup'),
        ('vacuum_pickup_approach', 'vacuum_pickup', 'vacuum_post_pickup'),
        ('vacuum_pickup_approach', 'vacuum_pickup', 'vacuum_post_pickup')
    ]

    for i, (approach, grasp, retreat) in enumerate(sample_locations, 1):
        print(f"\n  Processing sample {i}/3...")

        # Pick
        json_file = builder.pick_sequence(approach, grasp, retreat, gripper='epick')
        yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

        # Place
        json_file = builder.place_sequence('vacuum_place_approach', 'vacuum_place', 'vacuum_post_pickup', gripper='epick')
        yield from bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP})

        print(f"  ✓ Sample {i} complete")

    print("\n✅ All samples processed!")


# ============================================================
# MAIN: Run examples
# ============================================================

if __name__ == '__main__':
    print("\n" + "="*60)
    print("TaskBuilder Examples")
    print("="*60)

    # Show available locations
    builder.list_locations()

    print("\n\n" + "="*60)
    print("Which example would you like to run?")
    print("="*60)
    print("1. Simple move (move_to)")
    print("2. Pick sequence")
    print("3. Custom workflow (vision + pick + place)")
    print("4. Tool change")
    print("5. Multi-sample processing")
    print("0. Exit")

    try:
        choice = input("\nChoice (0-5): ").strip()

        if choice == '1':
            RE(new_way_simple_move())
        elif choice == '2':
            RE(new_way_pick_sequence())
        elif choice == '3':
            RE(new_way_custom_workflow())
        elif choice == '4':
            RE(new_way_tool_change())
        elif choice == '5':
            RE(new_way_multi_sample())
        elif choice == '0':
            print("Exiting...")
        else:
            print("Invalid choice")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        print("\n✓ Done!")
