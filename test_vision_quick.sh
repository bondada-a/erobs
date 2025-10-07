#!/bin/bash

# Quick test script for vision simulation
echo "Starting quick vision simulation test..."
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Step 1: Launching mock AprilTag detector...${NC}"
ros2 run mtc_pipeline mock_apriltag_detector &
MOCK_PID=$!
sleep 2

echo -e "${YELLOW}Step 2: Checking if detections are being published...${NC}"
timeout 3 ros2 topic echo /apriltag/detections --once

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Mock detector is working!${NC}"
else
    echo -e "${RED}✗ Mock detector not publishing${NC}"
fi

echo ""
echo -e "${YELLOW}Step 3: Checking TF transforms...${NC}"
timeout 2 ros2 run tf2_ros tf2_echo base_link tag36h11:0 --once

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ TF transforms are being published!${NC}"
else
    echo -e "${RED}✗ TF transforms not available${NC}"
fi

echo ""
echo -e "${YELLOW}Cleaning up...${NC}"
kill $MOCK_PID 2>/dev/null

echo -e "${GREEN}Test complete!${NC}"
echo ""
echo "To run full simulation:"
echo "1. Terminal 1: ros2 launch erobs_moveit_interface_sim demo.launch.py"
echo "2. Terminal 2: ros2 launch mtc_pipeline vision_system_sim.launch.py"
echo "3. Terminal 3: ros2 launch mtc_pipeline mtc_orchestrator.launch.py"
echo "4. Terminal 4: python3 src/mtc_pipeline/scripts/test_vision_sim.py --demo"