# Testing Patterns

**Analysis Date:** 2026-01-17

## Test Framework

**Runner:**
- pytest with ROS2 ament_lint integration
- ament_lint_auto for automatic linting test discovery

**Assertion Library:**
- pytest built-in assert statements
- ament_flake8, ament_pep257 for style checks

**Run Commands:**
```bash
colcon build                              # Build all packages
colcon test --packages-select beambot     # Run tests for beambot
colcon test-result --verbose              # View test results

# Manual test scripts (requires running beambot_bringup)
ros2 run beambot test_contour_detection   # Interactive contour testing
```

## Test File Organization

**Location:**
- Linting tests: `test/` directory within each package
- Manual tests: `scripts/` directory in beambot package

**Naming:**
- Linting: `test_flake8.py`, `test_pep257.py`, `test_copyright.py`
- Manual: `test_*.py` (e.g., `test_contour_detection.py`)

**Structure:**
```
src/beambot/
├── test/                           # Linting tests (if present)
│   ├── test_flake8.py
│   └── test_pep257.py
└── scripts/                        # Manual/integration tests
    ├── test_contour_detection.py
    └── test_wafer_detection.py

src/end_effectors/pipettor/pipette_driver/
└── test/
    ├── test_flake8.py
    ├── test_pep257.py
    └── test_copyright.py
```

## Test Structure

**Linting Test Pattern:**
```python
from ament_flake8.main import main_with_errors
import pytest

@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
```

**PEP257 Test Pattern:**
```python
from ament_pep257.main import main
import pytest

@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
```

**Patterns:**
- Use pytest markers for categorization (`@pytest.mark.linter`)
- ament tools provide main functions for style checking
- No setUp/tearDown - tests are stateless

## Mocking

**Framework:**
- Not currently used (no mock infrastructure)
- Manual tests interact with real/simulated hardware

**What to Mock (if implementing):**
- Zivid camera capture (slow, hardware-dependent)
- UR robot communication
- Action server responses

**What NOT to Mock:**
- MTC stage logic (needs real MoveIt)
- TF2 transforms (integral to system)

## Fixtures and Factories

**Test Data:**
- No pytest fixtures currently defined
- No conftest.py in test directories
- Test scripts create own ROS2 nodes inline

**Current Pattern (manual tests):**
```python
class ContourDetectionTest(Node):
    def __init__(self):
        super().__init__('contour_detection_test')
        # Setup subscriptions, parameters
        self.params = ContourDetectionParams()
```

## Coverage

**Requirements:**
- No enforced coverage target
- Linting is comprehensive (style checks on all files)
- Unit test coverage: Minimal

**Configuration:**
- No coverage tool configured
- Focus on integration testing via ROS2 actions

**View Coverage:**
```bash
# Not currently available - would need pytest-cov setup
```

## Test Types

**Linting Tests (Automated):**
- Scope: Code style and docstring validation
- Tools: ament_flake8, ament_pep257, ament_copyright
- Speed: Fast (<1s per package)
- Coverage: All Python files in package

**Manual/Integration Tests:**
- Scope: End-to-end vision and detection testing
- Location: `src/beambot/scripts/test_*.py`
- Speed: Slow (requires hardware or simulation)
- Execution: `ros2 run` with beambot_bringup running

**Launch Testing:**
- Not currently implemented
- Would use `launch_testing` package for launch file validation

**E2E Tests:**
- Not currently implemented
- Manual testing via GUI or Bluesky scripts

## Common Patterns

**Interactive Testing:**
```python
# Test scripts use keyboard input for parameter tuning
def spin_with_keyboard(self):
    """Spin with keyboard input for interactive testing."""
    while rclpy.ok():
        rclpy.spin_once(self, timeout_sec=0.1)
        # Check for keyboard input, adjust parameters
```

**ROS2 Node Testing:**
```python
def main(args=None):
    rclpy.init(args=args)
    test_node = ContourDetectionTest()
    try:
        test_node.spin_with_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
```

## Build Testing Integration

**CMakeLists.txt Pattern:**
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

**Package Dependencies:**
```xml
<test_depend>pytest</test_depend>
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

## Known Testing Gaps

**Critical Gaps:**
1. No unit tests for orchestrator task parsing logic
2. No unit tests for stage composition
3. No unit tests for gripper state tracking
4. No integration tests for tool exchange + motion sequence

**Missing Infrastructure:**
- No pytest fixtures or conftest.py
- No mock infrastructure for hardware
- No CI/CD pipeline for automated testing

**Recommended Additions:**
1. Unit tests for `orchestrator._parse_goal()` (JSON parsing)
2. Unit tests for `_group_into_batches()` (batching logic)
3. Integration tests with `use_fake_hardware:=true`
4. pytest-cov for coverage tracking

## Test Execution Workflow

**Development Testing:**
```bash
# 1. Build the workspace
colcon build

# 2. Run linting tests
colcon test --packages-select beambot pipette_driver

# 3. View results
colcon test-result --verbose
```

**Manual Integration Testing:**
```bash
# Terminal 1: Start simulation
ros2 launch beambot beambot_bringup.launch.py use_fake_hardware:=true

# Terminal 2: Run test scripts
ros2 run beambot test_contour_detection
```

**Pre-Commit (Recommended):**
```bash
# Not currently implemented - would add:
# - flake8 check
# - black formatting
# - pytest unit tests
```

---

*Testing analysis: 2026-01-17*
*Update when test patterns change*
