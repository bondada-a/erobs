#!/bin/bash
set -e

sudo apt-get update
command -v vcs >/dev/null 2>&1 || sudo apt-get install -y python3-vcstool

vcs import src/end_effectors < src/end_effectors/end_effectors.repos
vcs import src/vision < src/vision/vision.repos
rosdep update
rosdep install --from-paths src --ignore-src -y
