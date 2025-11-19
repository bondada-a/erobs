#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/aditya/work/github_ws/erobs')
from analyze_tool_voltage import analyze_bag

if __name__ == "__main__":
    analyze_bag("/home/aditya/work/github_ws/erobs/recorded_data/tool_voltage_issue_test")
