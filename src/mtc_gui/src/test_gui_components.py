#!/usr/bin/env python3

"""
Test script for MTC GUI components
"""

import sys
import os

# Add the current directory to Python path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test if all GUI components can be imported"""
    print("Testing GUI component imports...")
    
    try:
        from pose_editor import PoseManager
        print("✓ pose_editor imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import pose_editor: {e}")
    
    try:
        from poses_manager import PosesManager
        print("✓ poses_manager imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import poses_manager: {e}")
    
    try:
        from mtc_gui_client import MTCGUIClient
        print("✓ mtc_gui_client imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import mtc_gui_client: {e}")

def test_basic_functionality():
    """Test basic functionality of GUI components"""
    print("\nTesting basic functionality...")
    
    try:
        # Test pose editor
        from pose_editor import PoseManager
        print("✓ PoseManager class available")
    except Exception as e:
        print(f"✗ PoseManager test failed: {e}")
    
    try:
        # Test poses manager
        from poses_manager import PosesManager
        print("✓ PosesManager class available")
    except Exception as e:
        print(f"✗ PosesManager test failed: {e}")
    
    try:
        # Test main GUI client
        from mtc_gui_client import MTCGUIClient
        print("✓ MTCGUIClient class available")
    except Exception as e:
        print(f"✗ MTCGUIClient test failed: {e}")

def main():
    """Main test function"""
    print("=== MTC GUI Package Test ===\n")
    
    test_imports()
    test_basic_functionality()
    
    print("\n=== Test Complete ===")

if __name__ == '__main__':
    main()

