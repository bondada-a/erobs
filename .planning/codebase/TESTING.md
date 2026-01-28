# Testing Patterns

**Analysis Date:** 2026-01-27

## Test Framework

**Python:**
- Pytest - Primary framework (`test_depend` in `src/beambot/package.xml`)
- ament_lint_auto - Code quality enforcement
- ament_flake8, ament_pep257 - Style checking

**C++:**
- Google Test (GTest) - Unit tests (`src/end_effectors/*/tests/`)
- Google Mock - Mocking serial interfaces

**Run Commands:**
```bash
colcon build                      # Build all packages
colcon test                       # Run all tests
colcon test --packages-select beambot  # Single package
colcon test-result --verbose      # View results
```

## Test File Organization

**Location:**
- Python: `test/` directory or `scripts/test_*.py`
- C++: `test/` or `tests/` directories

**Naming:**
- Python: `test_*.py` or `*_test.py`
- C++: `test_*.cpp` or `*_test.cpp`

**Structure:**
```
src/beambot/
  scripts/
    test_wafer_detection.py       # Vision validation script
    test_contour_detection.py     # Contour detection test
    test_pointcloud_stability.py  # Point cloud tests

src/end_effectors/ros2_epick_gripper/
  epick_driver/tests/
    test_default_driver.cpp       # Driver unit tests
    test_data_utils.cpp           # Utility tests
    mock/mock_serial.hpp          # Mock interfaces
```

## Test Structure

**Python (Pytest style):**
```python
def test_detection_returns_valid_pose():
    # arrange
    detector = create_detector()

    # act
    result = detector.detect_markers([5])

    # assert
    assert result.success
    assert result.pose is not None
```

**C++ (GTest style):**
```cpp
TEST(TestDefaultDriver, activate) {
    MockSerial mock;
    EXPECT_CALL(mock, write(_)).Times(1);

    DefaultDriver driver(&mock);
    driver.activate();
}
```

**Patterns:**
- Arrange/Act/Assert structure
- One assertion focus per test
- Mocks for hardware interfaces

## Mocking

**Framework:**
- Python: Not extensively used (hardware tests are manual)
- C++: Google Mock for serial interfaces

**Patterns (C++):**
```cpp
#include <mock/mock_serial.hpp>

using ::testing::_;
using ::testing::Return;

TEST(TestDriver, send_command) {
    MockSerial mock;
    EXPECT_CALL(mock, write(_)).WillOnce(Return(true));

    Driver driver(&mock);
    ASSERT_TRUE(driver.send_command());
}
```

**What to Mock:**
- Serial/socket communication
- Hardware interfaces
- External services

**What NOT to Mock:**
- ROS2 node functionality (use integration tests)
- MTC planning (use simulation)

## Fixtures and Factories

**Test Data:**
```python
# Factory functions in test file
def create_test_goal():
    goal = MoveToAction.Goal()
    goal.target = "home"
    goal.poses_json = '{"home": [0, -90, 90, -90, -90, 0]}'
    return goal
```

**Location:**
- Inline in test files (no shared fixtures directory)
- C++ mock headers in `mock/` subdirectory

## Coverage

**Requirements:**
- No enforced coverage target
- Focus on critical paths (drivers, stages)

**Configuration:**
- ament_lint_auto finds test dependencies
- CMakeLists.txt enables testing via `BUILD_TESTING`

**View Coverage:**
```bash
# Coverage not configured by default
# Would need pytest-cov setup
```

## Test Types

**Unit Tests:**
- C++ driver tests: Hardware protocol verification
- Located in `src/end_effectors/*/tests/`
- Mock all external dependencies

**Integration Tests:**
- Launch tests via ROS2 testing framework
- Located with launch files
- Test node startup and communication

**Manual Validation Scripts:**
- Vision detection scripts: `src/beambot/scripts/test_*.py`
- Require camera hardware
- Human verification of results

## Common Patterns

**Launch Testing:**
```python
# In test_launch.py
import launch_testing
import pytest

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(package='beambot', executable='move_to_server'),
        launch_testing.actions.ReadyToTest()
    ])
```

**Async Testing:**
```python
async def test_action_client():
    node = rclpy.create_node('test_node')
    client = ActionClient(node, MoveToAction, 'beambot_moveto')

    goal = create_test_goal()
    future = client.send_goal_async(goal)

    result = await future
    assert result.accepted
```

**Error Testing:**
```python
def test_invalid_json_returns_error():
    goal = MoveToAction.Goal()
    goal.poses_json = "not valid json"

    result = stages.run(goal)
    assert not result.success
    assert "JSON" in result.error_message
```

**Hardware Mocking (C++):**
```cpp
// Mock serial interface
class MockSerial : public SerialInterface {
public:
    MOCK_METHOD(bool, write, (const std::vector<uint8_t>&), (override));
    MOCK_METHOD(std::vector<uint8_t>, read, (size_t), (override));
};
```

## Test Gaps

**Known Gaps:**
- No unit tests for orchestrator (1108 lines untested)
- No unit tests for stage compositions
- Vision tests are manual scripts, not automated
- No CI/CD pipeline configured

**Priority to Add:**
1. Orchestrator batch grouping logic
2. Stage composition with mock MTC
3. Configuration validation

---

*Testing analysis: 2026-01-27*
*Update when test patterns change*
