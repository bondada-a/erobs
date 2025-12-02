#!/usr/bin/env python3
"""Quick test script for local Bluesky installation"""

import sys

def test_imports():
    """Test that all required packages can be imported"""
    print("Testing Bluesky/ROS imports...")

    tests = []

    # Test Bluesky
    try:
        import bluesky
        print(f"✓ bluesky {bluesky.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ bluesky: {e}")
        tests.append(False)

    # Test Ophyd
    try:
        import ophyd
        print(f"✓ ophyd {ophyd.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ ophyd: {e}")
        tests.append(False)

    # Test Ophyd-Async
    try:
        import ophyd_async
        print(f"✓ ophyd_async {ophyd_async.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ ophyd_async: {e}")
        tests.append(False)

    # Test ROS 2
    try:
        import rclpy
        print(f"✓ rclpy (ROS 2 Humble)")
        tests.append(True)
    except ImportError as e:
        print(f"✗ rclpy: {e}")
        tests.append(False)

    # Test IPython
    try:
        import IPython
        print(f"✓ IPython {IPython.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ IPython: {e}")
        tests.append(False)

    # Test Tiled
    try:
        import tiled
        print(f"✓ tiled {tiled.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ tiled: {e}")
        tests.append(False)

    # Test Databroker
    try:
        import databroker
        print(f"✓ databroker {databroker.__version__}")
        tests.append(True)
    except ImportError as e:
        print(f"✗ databroker: {e}")
        tests.append(False)

    return all(tests)


def test_bluesky_basics():
    """Test basic Bluesky functionality"""
    print("\nTesting Bluesky basics...")

    try:
        from bluesky import RunEngine
        from bluesky.plans import count
        from ophyd.sim import det

        RE = RunEngine({})

        # Run a simple simulated scan
        print("Running a simple scan with simulated detector...")
        RE(count([det], num=5))

        print("✓ Bluesky RunEngine works!")
        return True
    except Exception as e:
        print(f"✗ Bluesky test failed: {e}")
        return False


def test_ros_init():
    """Test ROS 2 initialization"""
    print("\nTesting ROS 2 initialization...")

    try:
        import rclpy

        # Initialize ROS 2
        rclpy.init()

        # Create a simple node
        from rclpy.node import Node
        node = Node('test_node')
        print("✓ ROS 2 node created successfully!")

        # Clean up
        node.destroy_node()
        rclpy.shutdown()

        return True
    except Exception as e:
        print(f"✗ ROS 2 test failed: {e}")
        try:
            rclpy.shutdown()
        except:
            pass
        return False


def main():
    """Run all tests"""
    print("=" * 60)
    print("Bluesky/ROS Local Installation Test")
    print("=" * 60)

    results = []

    # Test 1: Imports
    print("\n--- Test 1: Package Imports ---")
    results.append(test_imports())

    # Test 2: Bluesky basics
    print("\n--- Test 2: Bluesky Functionality ---")
    results.append(test_bluesky_basics())

    # Test 3: ROS 2
    print("\n--- Test 3: ROS 2 Integration ---")
    results.append(test_ros_init())

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if all(results):
        print("✓ All tests passed! Your Bluesky/ROS environment is ready.")
        return 0
    else:
        print("✗ Some tests failed. Please check the errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
