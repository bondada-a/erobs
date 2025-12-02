#!/usr/bin/env python3
"""Test script for async MTC device

This script demonstrates and tests the async capabilities of the
MTCExecutionDeviceAsync compared to the original blocking device.
"""

import time
import rclpy
from bluesky import RunEngine
import bluesky.plan_stubs as bps

def test_async_device():
    """Test the async device implementation"""

    print("=" * 60)
    print("Async Device Test Suite")
    print("=" * 60)
    print()

    # Initialize ROS
    print("Initializing ROS 2...")
    rclpy.init()
    print("✓ ROS 2 initialized")
    print()

    # Import the async device
    from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync

    # Create RunEngine
    RE = RunEngine({})

    # Get robot IP from environment or use default
    import os
    ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.56.101")

    # Create robot device
    print(f"Creating async robot device (IP: {ROBOT_IP})...")
    robot = MTCExecutionDeviceAsync(name="ur5e_robot", robot_ip=ROBOT_IP)
    print("✓ Robot device created")
    print()

    # Test 1: Blocking execution (wait=True)
    print("-" * 60)
    print("Test 1: Blocking Execution (wait=True)")
    print("-" * 60)

    task_file = input("Enter path to test JSON file (or press Enter for default): ").strip()
    if not task_file:
        task_file = "task_sequences/complete_sequence.json"

    print(f"Executing: {task_file}")
    print("This should block until task completes...")
    print()

    start = time.time()
    try:
        RE(bps.abs_set(robot, task_file, wait=True))
        elapsed = time.time() - start
        print()
        print(f"✓ Test 1 PASSED: Task completed in {elapsed:.1f}s")
    except Exception as e:
        print(f"✗ Test 1 FAILED: {e}")

    print()
    input("Press Enter to continue to Test 2...")
    print()

    # Test 2: Non-blocking execution (wait=False)
    print("-" * 60)
    print("Test 2: Non-blocking Execution (wait=False)")
    print("-" * 60)
    print(f"Executing: {task_file}")
    print("This should return IMMEDIATELY...")
    print()

    start = time.time()
    try:
        status = RE(bps.abs_set(robot, task_file, wait=False))
        elapsed = time.time() - start

        print(f"✓ Returned in {elapsed:.2f}s")
        print(f"  Status done? {status.done}")
        print(f"  Status success? {status.success if status.done else 'Still running'}")

        if elapsed < 2.0:
            print(f"✓ Test 2 PASSED: Returned quickly ({elapsed:.2f}s < 2.0s)")
            print()
            print("Task is running in background...")
            print("Monitoring status for 10 seconds...")

            for i in range(10):
                time.sleep(1)
                if status.done:
                    print(f"  Status completed at {i+1}s")
                    break
                print(f"  {i+1}s: Still running...")

            if not status.done:
                print()
                print("Task still running after 10s (normal for long tasks)")

                if input("Cancel the task? (y/n): ").lower() == 'y':
                    print("Canceling task...")
                    robot.cancel_goal()
                    time.sleep(2)
                    print(f"Status after cancel: done={status.done}, success={status.success}")
        else:
            print(f"✗ Test 2 FAILED: Took too long to return ({elapsed:.2f}s)")

    except Exception as e:
        print(f"✗ Test 2 FAILED: {e}")

    print()
    print("-" * 60)
    print("Test 3: Cancellation")
    print("-" * 60)

    if input("Run cancellation test? (y/n): ").lower() == 'y':
        print(f"Starting task: {task_file}")
        print("Will cancel after 5 seconds...")
        print()

        try:
            status = RE(bps.abs_set(robot, task_file, wait=False))
            print("Task started, waiting 5 seconds...")
            time.sleep(5)

            print("Sending cancel request...")
            robot.cancel_goal()

            print("Waiting for cancellation to complete...")
            time.sleep(3)

            print(f"Status: done={status.done}, success={status.success}")

            if status.done and not status.success:
                print("✓ Test 3 PASSED: Task was canceled")
            else:
                print("⚠ Test 3 UNCLEAR: Check robot state")

        except Exception as e:
            print(f"✗ Test 3 FAILED: {e}")

    print()
    print("=" * 60)
    print("Test Suite Complete")
    print("=" * 60)
    print()

    # Cleanup
    print("Cleaning up...")
    rclpy.shutdown()
    print("✓ Done!")


def compare_devices():
    """Quick comparison between blocking and async devices"""

    print("=" * 60)
    print("Device Comparison: Blocking vs Async")
    print("=" * 60)
    print()

    rclpy.init()
    RE = RunEngine({})

    import os
    ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.56.101")
    task_file = "task_sequences/complete_sequence.json"

    print("Note: This will run the same task twice")
    print("      (once with each device)")
    print()

    if input("Proceed? (y/n): ").lower() != 'y':
        print("Aborted.")
        rclpy.shutdown()
        return

    # Test blocking device
    print()
    print("Testing BLOCKING device with wait=False:")
    print("-" * 60)

    from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice
    robot_blocking = MTCExecutionDevice(name="robot_blocking", robot_ip=ROBOT_IP)

    start = time.time()
    RE(bps.abs_set(robot_blocking, task_file, wait=False))
    elapsed_blocking = time.time() - start

    print(f"Returned in: {elapsed_blocking:.2f}s")
    print()

    time.sleep(2)  # Brief pause between tests

    # Test async device
    print("Testing ASYNC device with wait=False:")
    print("-" * 60)

    from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync
    robot_async = MTCExecutionDeviceAsync(name="robot_async", robot_ip=ROBOT_IP)

    start = time.time()
    status = RE(bps.abs_set(robot_async, task_file, wait=False))
    elapsed_async = time.time() - start

    print(f"Returned in: {elapsed_async:.2f}s")
    print()

    # Comparison
    print("=" * 60)
    print("Results:")
    print("=" * 60)
    print(f"Blocking device: {elapsed_blocking:.2f}s")
    print(f"Async device:    {elapsed_async:.2f}s")
    print()

    if elapsed_async < 2.0 and elapsed_blocking > elapsed_async:
        print("✓ ASYNC DEVICE WORKS!")
        print("  The async device returns immediately while the blocking device waits.")
    else:
        print("⚠ Results unclear - check implementation")

    print()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys

    print()
    print("Async Device Test Script")
    print()
    print("Options:")
    print("  1. Full test suite (recommended)")
    print("  2. Quick comparison (blocking vs async)")
    print()

    choice = input("Choose (1/2): ").strip()

    if choice == "2":
        compare_devices()
    else:
        test_async_device()
