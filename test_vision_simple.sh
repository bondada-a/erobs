#!/bin/bash
# Simple test script for vision system

TAG_ID=${1:-0}  # Default to tag 0

echo "Testing vision system: detect and move to tag $TAG_ID"

ros2 action send_goal /vision_move_to_action mtc_pipeline/action/VisionMoveToAction "{tag_id: $TAG_ID, timeout: 10.0}"
