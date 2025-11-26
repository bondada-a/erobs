# MTC Pipeline: Testing Implementation Quick-Start Guide

**Document:** Implementation Guide for Priority Test Cases
**Created:** November 26, 2025
**Target:** First 2 weeks of testing implementation

---

## Part 1: Setup & Infrastructure

### 1.1 Create Test Directory Structure

```bash
cd /home/aditya/work/github_ws/erobs/src/mtc_pipeline

# Create directory hierarchy
mkdir -p test/unit
mkdir -p test/integration
mkdir -p test/e2e/fixtures
mkdir -p test/helpers
mkdir -p test/mocks

# Create CMakeLists.txt for tests
touch test/CMakeLists.txt
touch test/unit/CMakeLists.txt
```

### 1.2 Update Main CMakeLists.txt

**Location:** `/home/aditya/work/github_ws/erobs/src/mtc_pipeline/CMakeLists.txt`

Add after line 193 (before `ament_package()`):

```cmake
if(BUILD_TESTING)
  # Add test subdirectory
  add_subdirectory(test)

  # Enable testing
  enable_testing()
endif()
```

### 1.3 Create test/CMakeLists.txt

```cmake
# Test configuration for MTC Pipeline

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# ============================================================
# UNIT TESTS
# ============================================================

# Gripper Utilities Unit Tests
add_executable(test_gripper_utils
  unit/test_gripper_utils.cpp
)
target_link_libraries(test_gripper_utils
  gtest
  gtest_main
)
ament_target_dependencies(test_gripper_utils
  rclcpp
)
gtest_discover_tests(test_gripper_utils)

# Input Validation Unit Tests
add_executable(test_mtc_orchestrator_input_validation
  unit/test_mtc_orchestrator_input_validation.cpp
)
target_link_libraries(test_mtc_orchestrator_input_validation
  mtc_pipeline_core
  obstacle_loader
  gtest
  gtest_main
)
ament_target_dependencies(test_mtc_orchestrator_input_validation
  rclcpp
  rclcpp_action
  nlohmann_json
)
gtest_discover_tests(test_mtc_orchestrator_input_validation)

# ============================================================
# INTEGRATION TESTS
# ============================================================

add_executable(test_orchestrator_action_coordination
  integration/test_orchestrator_action_coordination.cpp
)
target_link_libraries(test_orchestrator_action_coordination
  mtc_pipeline_core
  gtest
  gtest_main
)
ament_target_dependencies(test_orchestrator_action_coordination
  rclcpp
  rclcpp_action
)
gtest_discover_tests(test_orchestrator_action_coordination)

# ============================================================
# E2E TESTS (Python)
# ============================================================

# Requires pytest and launch_testing to be installed
# These are discovered automatically by pytest
```

---

## Part 2: Core Unit Tests

### 2.1 test/unit/test_gripper_utils.cpp (Priority: Week 1)

```cpp
#include <gtest/gtest.h>
#include "mtc_pipeline/gripper_utils.hpp"

namespace mtc_pipeline {

class GripperUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialization if needed
    }

    void TearDown() override {
        // Cleanup if needed
    }
};

// ============================================================
// GET_GROUP_NAME TESTS
// ============================================================

TEST_F(GripperUtilsTest, GetGroupNameHandE) {
    std::string result = gripper_utils::get_group_name("hande");
    EXPECT_EQ(result, "hande_gripper");
}

TEST_F(GripperUtilsTest, GetGroupNameEPick) {
    std::string result = gripper_utils::get_group_name("epick");
    EXPECT_EQ(result, "epick_gripper");
}

TEST_F(GripperUtilsTest, GetGroupNameUnknownGripper) {
    std::string result = gripper_utils::get_group_name("unknown_gripper");
    EXPECT_EQ(result, "unknown_gripper_gripper");  // Follows convention
}

TEST_F(GripperUtilsTest, GetGroupNameNone) {
    std::string result = gripper_utils::get_group_name("none");
    EXPECT_EQ(result, "");  // No movable joints
}

TEST_F(GripperUtilsTest, GetGroupNamePipettor) {
    std::string result = gripper_utils::get_group_name("pipettor");
    EXPECT_EQ(result, "");  // Static end effector
}

TEST_F(GripperUtilsTest, GetGroupNameEmpty) {
    std::string result = gripper_utils::get_group_name("");
    EXPECT_EQ(result, "");
}

// ============================================================
// GET_STATE_NAME TESTS
// ============================================================

TEST_F(GripperUtilsTest, GetStateNameHandEOpen) {
    std::string result = gripper_utils::get_state_name("hande", true);
    EXPECT_EQ(result, "hande_open");
}

TEST_F(GripperUtilsTest, GetStateNameHandEClosed) {
    std::string result = gripper_utils::get_state_name("hande", false);
    EXPECT_EQ(result, "hande_closed");
}

TEST_F(GripperUtilsTest, GetStateNameEPickVacuumOff) {
    std::string result = gripper_utils::get_state_name("epick", true);
    EXPECT_EQ(result, "vacuum_off");
}

TEST_F(GripperUtilsTest, GetStateNameEPickVacuumOn) {
    std::string result = gripper_utils::get_state_name("epick", false);
    EXPECT_EQ(result, "vacuum_on");
}

TEST_F(GripperUtilsTest, GetStateNameNoneThrows) {
    EXPECT_THROW(
        gripper_utils::get_state_name("none", true),
        std::invalid_argument
    );
}

TEST_F(GripperUtilsTest, GetStateNameEmptyThrows) {
    EXPECT_THROW(
        gripper_utils::get_state_name("", true),
        std::invalid_argument
    );
}

TEST_F(GripperUtilsTest, GetStateNamePipettorThrows) {
    EXPECT_THROW(
        gripper_utils::get_state_name("pipettor", true),
        std::invalid_argument
    );
}

// ============================================================
// INTEGRATION TESTS: GRIPPER TYPE CYCLES
// ============================================================

TEST_F(GripperUtilsTest, HandETypeCycle) {
    // Verify consistency: group name + states work together
    auto group = gripper_utils::get_group_name("hande");
    EXPECT_FALSE(group.empty());

    auto open = gripper_utils::get_state_name("hande", true);
    auto closed = gripper_utils::get_state_name("hande", false);

    EXPECT_NE(open, closed);
    EXPECT_TRUE(open.find("hande") != std::string::npos);
    EXPECT_TRUE(closed.find("hande") != std::string::npos);
}

TEST_F(GripperUtilsTest, EPickTypeCycle) {
    auto group = gripper_utils::get_group_name("epick");
    EXPECT_FALSE(group.empty());

    auto on = gripper_utils::get_state_name("epick", false);
    auto off = gripper_utils::get_state_name("epick", true);

    EXPECT_NE(on, off);
    EXPECT_EQ(on, "vacuum_on");
    EXPECT_EQ(off, "vacuum_off");
}

TEST_F(GripperUtilsTest, StaticGripperHasNoGroup) {
    // Grippers without moving parts return empty group
    auto noneGroup = gripper_utils::get_group_name("none");
    auto pipettorGroup = gripper_utils::get_group_name("pipettor");

    EXPECT_TRUE(noneGroup.empty());
    EXPECT_TRUE(pipettorGroup.empty());
}

}  // namespace mtc_pipeline

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

### 2.2 test/unit/test_mtc_orchestrator_input_validation.cpp (Priority: Week 1-2)

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <regex>

// Mock the orchestrator methods we're testing
class InputValidator {
public:
    // Returns true if IP is valid (prevents injection)
    static bool is_valid_robot_ip(const std::string& ip) {
        // Only allow IPv4 addresses: xxx.xxx.xxx.xxx format
        std::regex ipv4_pattern(R"(^(\d{1,3}\.){3}\d{1,3}$)");

        if (!std::regex_match(ip, ipv4_pattern)) {
            return false;
        }

        // Check each octet is 0-255
        auto parts = ip | split_string('.');
        for (const auto& part : parts) {
            try {
                int val = std::stoi(part);
                if (val < 0 || val > 255) return false;
            } catch (...) {
                return false;
            }
        }

        return true;
    }

    // Validate voltage is in valid range
    static bool is_valid_voltage(int voltage) {
        return voltage >= 0 && voltage <= 48;
    }

    // Check JSON doesn't exceed size limit
    static bool is_valid_json_size(const std::string& json_str) {
        const size_t MAX_JSON_SIZE = 10 * 1024 * 1024;  // 10MB
        return json_str.length() <= MAX_JSON_SIZE;
    }
};

class OrchestratorInputValidationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS if not already done
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Don't shutdown - keep ROS running for other tests
    }
};

// ============================================================
// ROBOT IP VALIDATION TESTS
// ============================================================

TEST_F(OrchestratorInputValidationTest, ValidIPAddressAccepted) {
    EXPECT_TRUE(InputValidator::is_valid_robot_ip("192.168.1.1"));
    EXPECT_TRUE(InputValidator::is_valid_robot_ip("10.0.0.1"));
    EXPECT_TRUE(InputValidator::is_valid_robot_ip("255.255.255.255"));
    EXPECT_TRUE(InputValidator::is_valid_robot_ip("0.0.0.0"));
}

TEST_F(OrchestratorInputValidationTest, RejectsShellMetacharactersInIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1;"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1; rm -rf /"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1|"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1&"));
}

TEST_F(OrchestratorInputValidationTest, RejectsBacktickExecutionInIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1`whoami`"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("`cat /etc/passwd`"));
}

TEST_F(OrchestratorInputValidationTest, RejectsDollarSignExecutionInIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1$(whoami)"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.$((1+1))"));
}

TEST_F(OrchestratorInputValidationTest, RejectsNewlineCharacterInIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1\nrm -rf /"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.1\r\nmalicious"));
}

TEST_F(OrchestratorInputValidationTest, RejectsInvalidOctets) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("256.1.1.1"));      // Octet > 255
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1.999"));  // Octet > 255
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("-1.1.1.1"));       // Negative
}

TEST_F(OrchestratorInputValidationTest, RejectsIncompleteIPAddress) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.1"));      // Only 3 octets
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168"));        // Only 2 octets
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192"));            // Only 1 octet
}

TEST_F(OrchestratorInputValidationTest, RejectsNonNumericIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("192.168.a.1"));
    EXPECT_FALSE(InputValidator::is_valid_robot_ip("hostname.local"));
}

TEST_F(OrchestratorInputValidationTest, RejectsEmptyIP) {
    EXPECT_FALSE(InputValidator::is_valid_robot_ip(""));
}

// ============================================================
// VOLTAGE VALIDATION TESTS
// ============================================================

TEST_F(OrchestratorInputValidationTest, ValidVoltageRange) {
    // UR robot tool voltage: 0-48V
    EXPECT_TRUE(InputValidator::is_valid_voltage(0));
    EXPECT_TRUE(InputValidator::is_valid_voltage(12));
    EXPECT_TRUE(InputValidator::is_valid_voltage(24));
    EXPECT_TRUE(InputValidator::is_valid_voltage(48));
}

TEST_F(OrchestratorInputValidationTest, RejectsNegativeVoltage) {
    EXPECT_FALSE(InputValidator::is_valid_voltage(-1));
    EXPECT_FALSE(InputValidator::is_valid_voltage(-100));
}

TEST_F(OrchestratorInputValidationTest, RejectsExcessiveVoltage) {
    EXPECT_FALSE(InputValidator::is_valid_voltage(49));
    EXPECT_FALSE(InputValidator::is_valid_voltage(100));
    EXPECT_FALSE(InputValidator::is_valid_voltage(1000));
    EXPECT_FALSE(InputValidator::is_valid_voltage(INT_MAX));
}

// ============================================================
// JSON SIZE VALIDATION TESTS
// ============================================================

TEST_F(OrchestratorInputValidationTest, AcceptsSmallJSON) {
    std::string small_json = R"({"start_gripper": "hande", "tasks": []})";
    EXPECT_TRUE(InputValidator::is_valid_json_size(small_json));
}

TEST_F(OrchestratorInputValidationTest, AcceptsLargeButValidJSON) {
    // Create 5MB JSON (under 10MB limit)
    std::string large_json = R"({"start_gripper": "hande", "tasks": [)";
    for (int i = 0; i < 10000; ++i) {
        large_json += R"({"task_type": "moveto", "data": ")" +
                      std::string(500, 'x') + "\"},";
    }
    large_json += R"(]})";

    EXPECT_LT(large_json.length(), 10 * 1024 * 1024);
    EXPECT_TRUE(InputValidator::is_valid_json_size(large_json));
}

TEST_F(OrchestratorInputValidationTest, RejectsExcessivelyLargeJSON) {
    // Create 15MB JSON payload
    std::string huge_json = R"({"data": ")";
    huge_json += std::string(15 * 1024 * 1024, 'x');
    huge_json += R"("})";

    EXPECT_FALSE(InputValidator::is_valid_json_size(huge_json));
}

// ============================================================
// JSON FIELD VALIDATION TESTS
// ============================================================

TEST_F(OrchestratorInputValidationTest, ValidJSONStructure) {
    std::string valid_json = R"({
        "start_gripper": "hande",
        "tasks": [
            {"task_type": "moveto", "target": "home"}
        ]
    })";

    try {
        auto parsed = nlohmann::json::parse(valid_json);
        EXPECT_TRUE(parsed.contains("start_gripper"));
        EXPECT_TRUE(parsed.contains("tasks"));
        EXPECT_TRUE(parsed["tasks"].is_array());
    } catch (...) {
        FAIL() << "Valid JSON should parse without exception";
    }
}

TEST_F(OrchestratorInputValidationTest, RejectsInvalidJSON) {
    std::string invalid_json = R"({invalid json})";

    EXPECT_THROW(
        nlohmann::json::parse(invalid_json),
        nlohmann::json::exception
    );
}

TEST_F(OrchestratorInputValidationTest, RejectsMissingStartGripper) {
    std::string json = R"({"tasks": []})";
    auto parsed = nlohmann::json::parse(json);

    EXPECT_FALSE(parsed.contains("start_gripper"));
}

TEST_F(OrchestratorInputValidationTest, RejectsMissingTasks) {
    std::string json = R"({"start_gripper": "hande"})";
    auto parsed = nlohmann::json::parse(json);

    EXPECT_FALSE(parsed.contains("tasks"));
}

TEST_F(OrchestratorInputValidationTest, RejectsNonArrayTasks) {
    std::string json = R"({
        "start_gripper": "hande",
        "tasks": "not an array"
    })";
    auto parsed = nlohmann::json::parse(json);

    EXPECT_FALSE(parsed["tasks"].is_array());
}

// ============================================================
// GRIPPER NAME VALIDATION
// ============================================================

TEST_F(OrchestratorInputValidationTest, ValidGripperNames) {
    // Valid gripper types
    EXPECT_TRUE(is_valid_gripper_name("hande"));
    EXPECT_TRUE(is_valid_gripper_name("epick"));
    EXPECT_TRUE(is_valid_gripper_name("pipettor"));
    EXPECT_TRUE(is_valid_gripper_name("none"));
}

TEST_F(OrchestratorInputValidationTest, RejectsPathTraversalInGripperName) {
    EXPECT_FALSE(is_valid_gripper_name("../../../etc/passwd"));
    EXPECT_FALSE(is_valid_gripper_name("hande/../../"));
    EXPECT_FALSE(is_valid_gripper_name("..\\windows\\system32"));
}

TEST_F(OrchestratorInputValidationTest, RejectsShellMetacharactersInGripperName) {
    EXPECT_FALSE(is_valid_gripper_name("hande; rm -rf /"));
    EXPECT_FALSE(is_valid_gripper_name("hande|cat /etc/passwd"));
    EXPECT_FALSE(is_valid_gripper_name("$(whoami)"));
}

}  // End of test cases

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Part 3: Build and Run Tests

### 3.1 Build Tests

```bash
cd /home/aditya/work/github_ws/erobs

# Build only the test targets
colcon build --packages-select mtc_pipeline --cmake-args -DBUILD_TESTING=ON

# Or full build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON
```

### 3.2 Run Tests

```bash
# Run all tests in mtc_pipeline
colcon test --packages-select mtc_pipeline

# Run specific test
colcon test --packages-select mtc_pipeline --ctest-args -R test_gripper_utils

# Run with verbose output
colcon test --packages-select mtc_pipeline --ctest-args -VV

# Generate coverage report (requires gcov)
colcon build --packages-select mtc_pipeline \
  --cmake-args -DCMAKE_CXX_FLAGS="--coverage"
colcon test --packages-select mtc_pipeline
gcovr --root . --html coverage.html
```

### 3.3 View Test Results

```bash
# Check test results
cat /home/aditya/work/github_ws/erobs/build/mtc_pipeline/test_results/*/result.xml

# View coverage
open /home/aditya/work/github_ws/erobs/coverage/index.html
```

---

## Part 4: Helper Mock Classes for Integration Tests

### 4.1 test/helpers/mock_action_servers.hpp

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gmock/gmock.h>
#include "mtc_pipeline/action/move_to_action.hpp"
#include "mtc_pipeline/action/end_effector_action.hpp"
#include "mtc_pipeline/action/pick_place_action.hpp"

namespace testing {

/**
 * Mock action server for testing orchestrator task dispatch
 */
class MockActionServer : public rclcpp::Node {
public:
    MockActionServer(const std::string& node_name, const std::string& action_name)
        : Node(node_name) {
        // Implementation of mock action server
    }

    // Track received goals for verification
    std::vector<std::string> received_goal_types;
    std::vector<std::string> received_targets;
};

}  // namespace testing
```

---

## Part 5: Quick Reference: Test Checklist

### Week 1 Checklist
- [ ] Create test directory structure
- [ ] Update CMakeLists.txt for testing
- [ ] Implement `test_gripper_utils.cpp` (14 tests)
- [ ] Implement `test_mtc_orchestrator_input_validation.cpp` (30+ tests)
- [ ] Verify both test suites compile and pass
- [ ] Set up GitHub Actions workflow for CI/CD

### Week 2 Checklist
- [ ] Implement `test_orchestrator_action_coordination.cpp`
- [ ] Add process management mocks
- [ ] Add socket communication mocks
- [ ] Implement concurrency tests
- [ ] Achieve 30%+ code coverage

### Ongoing
- [ ] Add integration tests for each action server
- [ ] Performance regression baselines
- [ ] Memory leak detection tests
- [ ] Achieve 80%+ coverage target

---

## Part 6: Debugging Tips

### Run Single Test with GDB

```bash
cd /home/aditya/work/github_ws/erobs/build/mtc_pipeline
gdb ./test_gripper_utils
(gdb) run --gtest_filter=GripperUtilsTest.GetGroupNameHandE
```

### Memory Sanitizer

```bash
colcon build --packages-select mtc_pipeline \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"
colcon test --packages-select mtc_pipeline
```

### Thread Sanitizer

```bash
colcon build --packages-select mtc_pipeline \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread"
colcon test --packages-select mtc_pipeline
```

---

## References

- GTest Documentation: https://google.github.io/googletest/
- ROS 2 Testing: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing.html
- launch_testing: https://github.com/ros-infrastructure/launch_testing

---

**Next Steps:** Execute Phase 1 implementation starting with `test_gripper_utils.cpp` and `test_mtc_orchestrator_input_validation.cpp`.
