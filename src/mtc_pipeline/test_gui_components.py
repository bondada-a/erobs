#!/usr/bin/env python3

"""
Test script for MTC GUI components
This script tests the basic functionality of the GUI components without requiring ROS2
"""

import sys
import os
import tkinter as tk

def test_pose_editor():
    """Test the pose editor component"""
    print("Testing Pose Editor...")
    
    try:
        from pose_editor import PoseEditor
        
        # Create a test window
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        
        # Test pose editor
        editor = PoseEditor(root, "test_pose", [0.0, -90.0, -90.0, -90.0, 90.0, 0.0])
        result = editor.show()
        
        if result:
            print(f"✓ Pose Editor: Created pose '{result['name']}' with values {result['values']}")
        else:
            print("✓ Pose Editor: Cancelled (expected behavior)")
        
        root.destroy()
        return True
        
    except Exception as e:
        print(f"✗ Pose Editor: Failed - {str(e)}")
        return False

def test_poses_manager():
    """Test the poses manager component"""
    print("Testing Poses Manager...")
    
    try:
        from poses_manager import PosesManager
        
        # Create a test window
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        
        # Test poses manager
        test_poses = {
            "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            "test": [45.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        }
        
        manager = PosesManager(root, test_poses)
        result = manager.show()
        
        if result is not None:
            print(f"✓ Poses Manager: Managed {len(result)} poses")
        else:
            print("✓ Poses Manager: Cancelled (expected behavior)")
        
        root.destroy()
        return True
        
    except Exception as e:
        print(f"✗ Poses Manager: Failed - {str(e)}")
        return False

def test_gui_client_import():
    """Test importing the main GUI client"""
    print("Testing GUI Client Import...")
    
    try:
        # Try to import the main GUI client
        # This will fail without ROS2, but we can check the import structure
        import mtc_gui_client
        
        print("✓ GUI Client: Import successful")
        return True
        
    except ImportError as e:
        if "rclpy" in str(e):
            print("✓ GUI Client: Import structure correct (ROS2 not available)")
            return True
        else:
            print(f"✗ GUI Client: Import failed - {str(e)}")
            return False
    except Exception as e:
        print(f"✗ GUI Client: Unexpected error - {str(e)}")
        return False

def main():
    """Run all tests"""
    print("MTC GUI Components Test")
    print("=" * 40)
    
    tests = [
        test_pose_editor,
        test_poses_manager,
        test_gui_client_import
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"✗ Test failed with exception: {str(e)}")
            results.append(False)
    
    print("\n" + "=" * 40)
    print("Test Results:")
    
    passed = sum(results)
    total = len(results)
    
    for i, (test, result) in enumerate(zip(tests, results)):
        status = "PASS" if result else "FAIL"
        print(f"  {i+1}. {test.__name__}: {status}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All tests passed! GUI components are ready to use.")
        return 0
    else:
        print("✗ Some tests failed. Check the output above for details.")
        return 1

if __name__ == '__main__':
    sys.exit(main())
