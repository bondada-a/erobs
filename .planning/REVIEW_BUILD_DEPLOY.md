# EROBS Build & Deploy Review

**Review Date:** 2025-01-29  
**Reviewed by:** Automated Build Analysis

---

## Executive Summary

This review covers all build and deployment files in the EROBS (Experimental Robotics for Beamline Science) project:
- **10 Dockerfiles** (7 active, 3 archived)
- **1 Docker Compose file**
- **21 Shell scripts**
- **20 CMakeLists.txt files**
- **20 package.xml files**
- **4 GitHub Actions workflows**
- **1 setup.py file**

### Overall Assessment: 🟡 MODERATE - Several improvements recommended

---

## Table of Contents
1. [Docker Analysis](#docker-analysis)
2. [Shell Script Analysis](#shell-script-analysis)
3. [CMake Analysis](#cmake-analysis)
4. [Package Dependencies](#package-dependencies)
5. [CI/CD Analysis](#cicd-analysis)
6. [Priority Recommendations](#priority-recommendations)

---

## Docker Analysis

### Active Dockerfiles

#### 1. `docker/erobs-common-img/Dockerfile` (Main beambot_img)
**Purpose:** Full ROS/robotics image with Zivid SDK  
**Base:** `osrf/ros:humble-desktop-full`  
**Size Impact:** Very large (~8-10GB estimated)

**Issues:**
- ❌ **No multi-stage build** - Final image contains all build tools
- ❌ **CACHEBUST invalidates entire clone** - Consider using `--mount=type=cache` for `apt`
- ⚠️ **Multiple RUN apt-get update** - Line 9, 33, 42, 67 - should combine
- ⚠️ **Hardcoded Zivid IP** (10.68.81.52) in Cameras.yml - should be runtime configurable
- ⚠️ **Hardcoded paths in sed** for Intel oneAPI setup
- ✅ Good: Clean apt cache at end
- ✅ Good: Uses `--no-cache` for fresh builds

**Recommendation:**
```dockerfile
# Combine apt operations
RUN apt-get update && apt-get install -y --no-install-recommends \
    package1 package2 ... \
    && rm -rf /var/lib/apt/lists/*
```

#### 2. `docker/beambot_img/Dockerfile` (Lightweight version)
**Purpose:** ROS image without Zivid SDK  
**Base:** `osrf/ros:humble-desktop-full`

**Issues:**
- ✅ Uses `--no-install-recommends` - good practice
- ❌ **Missing cleanup in pip install** - add `--no-cache-dir`
- ⚠️ **Duplicates VNC setup script** - same as erobs-common-img
- ⚠️ **No health check defined**

#### 3. `docker/bsui/Dockerfile` (Bluesky full)
**Purpose:** Full Bluesky/ROS integration image  
**Size:** ~5GB (as noted in img_build.sh)

**Issues:**
- ❌ **Conda installed AFTER ROS builds** - Comment explains why, but creates large final image
- ❌ **EPICS built from source** - Could use pre-built packages
- ⚠️ **No .dockerignore** visible - may include unnecessary files
- ⚠️ **Missing runtime runtime exec_depend** for python packages
- ✅ Good: Has cleanup step
- ✅ Good: Builds interface packages first

#### 4. `docker/bsui-minimal/Dockerfile` (Bluesky lightweight)
**Purpose:** Minimal Bluesky client  
**Size:** ~1.5GB (as noted)

**Issues:**
- ✅ Uses `--no-install-recommends`
- ✅ Uses `--no-cache-dir` for pip
- ⚠️ **COPY before build** - `beambot_interfaces` copied before rosdep, could invalidate cache
- ⚠️ **PYTHONPATH hardcoded** to `/ros2_ws/src`
- ✅ Good: Smaller footprint than full bsui

#### 5. `docker/ur-driver/Dockerfile`
**Purpose:** UR robot driver  

**Issues:**
- ❌ **Old repo URL** - Points to `nsls2/erobs` not `bondada-a/erobs`
- ❌ **No CACHEBUST** - Uses stale git clone
- ⚠️ **Hardcoded description package** in entrypoint
- ⚠️ **entrypoint.sh uses `.` instead of `source`** - works but less readable

#### 6. `docker/ur-moveit/Dockerfile`
**Purpose:** MoveIt with RViz  

**Issues:**
- ❌ **Old repo URL** - `nsls2/erobs`
- ❌ **Missing `--no-install-recommends`**
- ⚠️ **Separate RUN for x11vnc** - could combine with previous apt-get
- ⚠️ **Missing CACHEBUST** for git clone

#### 7. `docker/ursim/Dockerfile`
**Purpose:** UR Simulator  
**Base:** `universalrobots/ursim_e-series`

**Issues:**
- ✅ Very simple, minimal - good
- ⚠️ **URCap file path assumes local copy exists**

#### 8. `docker/ur-example/Dockerfile`
**Purpose:** Example/test container

**Issues:**
- ⚠️ **Minimal Dockerfile** - just runs test
- ✅ Simple and focused

#### 9. `docker/azure-kinect/Dockerfile.txt`
**Purpose:** Azure Kinect camera support  
**Note:** File extension `.txt` suggests may be deprecated

**Issues:**
- ❌ **Deprecated Microsoft repos** - Uses Ubuntu 18.04/20.04 repos on Humble (22.04)
- ❌ **Uses `sudo`** - Unnecessary in Docker (already root)
- ❌ **Uses `|| true` to ignore apt errors** - Hides real problems
- ⚠️ **Points to personal fork** (ChandimaFernando/erobs)
- ⚠️ **Mixing apt-key (deprecated) with keyring method**

#### 10. `.devcontainer/Dockerfile`
**Purpose:** VS Code devcontainer  
**Base:** `althack/ros2:humble-full`

**Issues:**
- ✅ Uses `--no-install-recommends`
- ✅ Installs Pixi for modern dependency management
- ✅ Switches user appropriately
- ⚠️ **WORKSPACE arg** used but not validated

### Archived Dockerfiles

Located in `docker/archive/` - should be deleted if no longer needed:
- `docker/archive/bsui/Dockerfile` - Uses GitHub token as build arg (security risk if pushed)
- `docker/archive/erobs-common-img/Dockerfile` - Old version

### Docker Compose (`docker/docker-compose.yml`)

**Issues:**
- ⚠️ **Version '3' deprecated** - Use `version: '3.8'` or remove entirely (compose v2)
- ⚠️ **Fixed IP addresses** - May conflict in different environments
- ⚠️ **No resource limits** defined
- ⚠️ **No restart policies**
- ✅ Good: Uses depends_on
- ✅ Good: Network isolation

---

## Shell Script Analysis

### Critical Scripts

#### 1. `build.sh`
```bash
#!/bin/bash
set -e
```

**Issues:**
- ⚠️ **Second colcon build overwrites cmake args** - `-Wall -Wextra -Wpedantic` passed wrong (should be in quotes)
- ⚠️ **No error context** - Could use `set -o pipefail`
- ⚠️ **Hardcoded package list** - May become stale

**Recommendation:**
```bash
#!/bin/bash
set -euo pipefail

colcon build \
    --merge-install \
    --symlink-install \
    --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
                 "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
                 "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic"
```

#### 2. `setup.sh`
```bash
#!/bin/bash
set -e
```

**Issues:**
- ⚠️ **Uses `sudo`** - Will fail in Docker or if user is root
- ⚠️ **No error handling for vcs import**

**Recommendation:**
```bash
#!/bin/bash
set -euo pipefail

vcs import < src/ros2.repos src || { echo "vcs import failed"; exit 1; }

# Handle sudo gracefully
if [ "$(id -u)" -eq 0 ]; then
    apt-get update
else
    sudo apt-get update
fi
```

#### 3. `test.sh`
```bash
#!/bin/bash
set -e
```

**Issues:**
- ✅ Good: Uses set -e
- ⚠️ **No timeout** for test execution

#### 4. `start_ursim.sh`
```bash
#!/bin/bash
```

**Issues:**
- ❌ **Missing `set -e`**
- ⚠️ **No error handling** if ros2 command fails

#### 5. `docker/img_build.sh`
```bash
#!/bin/bash
set -e
```

**Issues:**
- ✅ Good: Uses set -e
- ✅ Good: Clear usage message
- ⚠️ **Uses `--no-cache`** - Slow builds, consider BuildKit cache
- ⚠️ **REGISTRY hardcoded** to `ghcr.io/bondada-a`

### VNC Scripts (`start-vnc.sh` - 2 identical copies)

**Issues:**
- ⚠️ **Duplicate files** - `docker/erobs-common-img/start-vnc.sh` and `docker/beambot_img/start-vnc.sh` are identical
- ⚠️ **No error handling** if Xvfb or x11vnc fail
- ⚠️ **Background process PID saved but never used**
- ⚠️ **No graceful shutdown**

**Recommendation:** Move to single shared location, add error handling:
```bash
#!/bin/bash
set -euo pipefail

cleanup() {
    kill $XVFB_PID 2>/dev/null || true
}
trap cleanup EXIT

Xvfb :1 -screen 0 1920x1080x24 &
XVFB_PID=$!

sleep 2

if ! x11vnc -display :1 -rfbport 5901 -forever -shared -nopw -bg; then
    echo "Failed to start x11vnc"
    exit 1
fi
```

### PDF Launch Scripts (`scripts/pdf-launch-scripts/`)

**Issues found across all scripts:**
- ❌ **Missing shebang consistency** - Some scripts missing proper headers
- ❌ **Hardcoded IPs** (10.66.218.141, 10.66.218.39, etc.)
- ⚠️ **Mixing docker and podman** - `bsui-launch.sh` uses podman, others use docker
- ⚠️ **Some scripts marked "TO BE DEPRECATED"** but still present
- ⚠️ **Environment variables not quoted** - Could break with spaces

| Script | Issues |
|--------|--------|
| `bsui-launch.sh` | Uses podman, hardcoded CDC_LOCALHOST |
| `hello-talker.sh` | Simple, OK |
| `mtc-moveit-launch.sh` | Deprecated, hardcoded IPs |
| `robotiq-driver-entrypoint.sh` | Uses `&` backgrounding, fragile |
| `robotiq-driver-launch.sh` | Hardcoded IPs |
| `sample-movement-server-launch.sh` | Deprecated |
| `ur-driver-launch.sh` | Hardcoded IPs |
| `ur-hande-driver-launch.sh` | Deprecated |

### Other Scripts

#### `src/custom-ur-descriptions/ur5e_robot_description/urdf/convert_urdf_for_isaac.sh`

**Issues:**
- ❌ **Hardcoded home paths** (`/home/aditya/work/github_ws/experimental/...`)
- ⚠️ **sed replacements are fragile** - Will break if paths change
- ✅ Good: Has proper error checking and usage message

**Recommendation:** Use environment variables or auto-detect paths:
```bash
WORKSPACE_ROOT="${EROBS_WORKSPACE:-$(dirname $(dirname $0))}"
```

#### `src/bluesky_ros/archive/local_bsui.sh`

**Issues:**
- ✅ Good: Extensive documentation
- ✅ Good: Color output
- ✅ Good: Auto-detects workspace
- ⚠️ **Complex script** - Could be simplified

### GitHub Actions Scripts

#### `.github/actions/lint/run.sh`
```bash
#!/bin/bash
set -e
ament_${LINTER} src/
```

**Issues:**
- ⚠️ **Variable not quoted** - `${LINTER}` should be `"${LINTER}"`
- ⚠️ **No error message** on failure

#### `.github/actions/test/run.sh`
```bash
#!/bin/bash
set -e
./setup.sh
./build.sh
# ./test.sh  # Commented out!
```

**Issues:**
- ❌ **Tests commented out** - CI passes without running tests!
- ⚠️ **Missing `set -o pipefail`**

---

## CMake Analysis

### Summary of CMakeLists.txt Files

| Package | CMake Version | Build Type | Issues |
|---------|--------------|------------|--------|
| `hello_moveit` | 3.8 | ament_cmake | Has commented-out component code |
| `hello_orchestrator` | 3.8 | ament_cmake | Generates own actions (duplicate?) |
| `hello_orchestrator_interfaces` | 3.8 | ament_cmake | OK |
| `hello_moveit_interfaces` | 3.8 | ament_cmake | OK |
| `hello_orchestrator_py` | 3.8 | ament_cmake_python | OK |
| `hello_orchestrator_py_interfaces` | 3.8 | ament_cmake | OK |
| `beambot` | 3.8 | ament_cmake_python | OK |
| `beambot_interfaces` | 3.8 | ament_cmake | OK |
| `ur3e_hande_moveit_config` | 3.8 | ament_cmake | OK |
| `ur3e_hande_robot_description` | 3.8 | ament_cmake | OK |
| `ur_zivid_*_moveit_config` (4) | 3.22 | ament_cmake | Higher version required |
| `ur5e_robot_description` | 3.8 | ament_cmake | OK |
| `pdf_beamtime` | 3.8 | ament_cmake | Missing geometry_msgs in CMake deps |
| `pdf_beamtime_interfaces` | 3.8 | ament_cmake | OK |
| `epick_config` | 3.8 | ament_cmake | Uses C++17 |
| `aruco_pose` | 3.8 | ament_cmake | OK |

### Inconsistencies Found

1. **CMake Version Mismatch:**
   - Most packages: `cmake_minimum_required(VERSION 3.8)`
   - MoveIt configs: `cmake_minimum_required(VERSION 3.22)`
   - **Recommendation:** Standardize on 3.16+ for Humble compatibility

2. **Duplicate Action Definitions:**
   - `hello_orchestrator` generates `PrintMessage.action`, `MoveToNamedState.action`, `OrchestratorTask.action`
   - `hello_orchestrator_interfaces` generates the SAME actions
   - **Recommendation:** Remove duplication, use only `_interfaces` package

3. **Missing `ament_export_dependencies`:**
   - Interface packages should export runtime dependencies for downstream packages

4. **Copyright/License Skips:**
   - Most packages skip `ament_cmake_copyright` and `ament_cmake_cpplint`
   - These should be enabled once copyright headers are added

5. **C++ Standard Not Set:**
   - Only `epick_config` explicitly sets C++17
   - ROS 2 Humble defaults to C++17, but explicit is better

**Recommendation for all packages:**
```cmake
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
```

---

## Package Dependencies

### Missing Dependencies Found

| Package | Missing From | Should Have |
|---------|-------------|-------------|
| `aruco_pose` | package.xml | `message_filters` as `depend` (has as buildtool) |
| `pdf_beamtime` | CMakeLists.txt | `geometry_msgs` in dependencies list |
| `hello_moveit` | package.xml | `geometry_msgs` if poses are used |
| `mtc_gui` | package.xml | Missing `action_msgs` for action usage |

### Unused Dependencies

| Package | Potentially Unused |
|---------|-------------------|
| `beambot` | `std_srvs` - check if actually used |
| `hello_orchestrator_py` | `moveit_task_constructor_core` - is it used in Python? |

### Dependency Graph Issues

1. **Interface packages build order:**
   - `beambot_interfaces` must build before `beambot`
   - `hello_moveit_interfaces` must build before `hello_moveit`
   - Build scripts handle this, but could use `colcon` package selection better

2. **External Dependencies (rosdep):**
   - `zivid_interfaces` - skipped in rosdep, built locally
   - `pipette_driver` - skipped in rosdep, built locally
   - These skip-keys are documented in Dockerfiles ✅

### Version Constraints

None of the package.xml files specify version constraints. Consider adding for critical dependencies:
```xml
<depend version_gte="2.0.0">rclpy</depend>
```

---

## CI/CD Analysis

### GitHub Actions Workflows

#### 1. `ros.yaml` - ROS C++ Testing and Linting
**Triggers:** push/PR to main, humble

**Issues:**
- ❌ **Tests are disabled** in `.github/actions/test/run.sh`
- ⚠️ **No caching** for apt or ROS dependencies
- ⚠️ **No matrix for different ROS distros**
- ✅ Good: Matrix linting (cppcheck, uncrustify, lint_cmake, cpplint, xmllint)

#### 2. `ruff.yml` - Python Linting
**Triggers:** push/PR to main, humble

**Issues:**
- ⚠️ **Uses `--fix`** which modifies code in CI (should be check-only)
- ⚠️ **No commit of fixed code** - changes are lost
- ✅ Good: Modern Python linter

**Recommendation:**
```yaml
- name: Ruff Check
  run: ruff check --diff  # Show diff, don't modify
- name: Ruff Format Check
  run: ruff format --check
```

#### 3. `super-linter.yml` - Multi-language Linting
**Triggers:** push/PR to main, humble

**Issues:**
- ✅ Good: Validates YAML, XML, JSON, Markdown
- ✅ Good: Gitleaks for secrets detection
- ⚠️ **JSCPD disabled** (copy-paste detection) - TODO in file
- ⚠️ **Only validates changed files** (`VALIDATE_ALL_CODEBASE: false`)

#### 4. `docker-publish.yml` - Docker Image Publishing
**Triggers:** Manual (`workflow_dispatch`)

**Issues:**
- ⚠️ **Manual only** - Not triggered on releases or main branch
- ⚠️ **Missing `beambot_bsui_minimal`** in options
- ✅ Good: Uses BuildKit cache
- ✅ Good: Frees disk space before build
- ✅ Good: Tags with SHA and user-specified tag

**Recommendation:** Add automatic trigger:
```yaml
on:
  workflow_dispatch:
    ...
  push:
    branches: [main]
    paths:
      - 'docker/**'
```

### Missing CI/CD Components

1. **No automated tests** - Tests are commented out
2. **No release workflow** - No semantic versioning or changelog
3. **No dependency caching** - Slow builds
4. **No security scanning** - Could add Trivy for container scanning
5. **No documentation generation** - Could use sphinx or mkdocs

---

## Priority Recommendations

### 🔴 Critical (Fix Immediately)

1. **Enable tests in CI:**
   ```bash
   # .github/actions/test/run.sh
   ./test.sh  # Uncomment this line!
   ```

2. **Remove hardcoded secrets/paths:**
   - Remove GITHUB_TOKEN from archived Dockerfile
   - Parameterize hardcoded IP addresses
   - Remove hardcoded user paths (`/home/aditya/...`)

3. **Fix deprecated Docker practices:**
   - Update `docker-compose.yml` version
   - Remove `sudo` from Dockerfiles
   - Fix Ubuntu repo issues in Azure Kinect Dockerfile

### 🟡 Important (Fix Soon)

4. **Consolidate duplicate files:**
   - Merge `start-vnc.sh` scripts
   - Remove duplicate action definitions in `hello_orchestrator`

5. **Add error handling to scripts:**
   ```bash
   set -euo pipefail
   ```

6. **Update old repository URLs:**
   - `nsls2/erobs` → `bondada-a/erobs`

7. **Enable ruff check-only mode in CI**

8. **Add CACHEBUST or version pinning to git clones**

### 🟢 Nice to Have (Improve Over Time)

9. **Multi-stage Docker builds** to reduce image size

10. **Add health checks** to Docker services

11. **Enable dependency caching** in GitHub Actions:
    ```yaml
    - uses: actions/cache@v4
      with:
        path: ~/.cache/pip
        key: ${{ runner.os }}-pip-${{ hashFiles('**/requirements.txt') }}
    ```

12. **Add container security scanning** (Trivy)

13. **Create `.dockerignore`** file:
    ```
    .git
    .github
    *.md
    build/
    install/
    log/
    ```

14. **Standardize CMake versions** across packages

15. **Add version constraints** to package.xml dependencies

16. **Clean up deprecated scripts** in `pdf-launch-scripts/`

---

## Files Inventory

### Dockerfiles (10 total)
```
docker/erobs-common-img/Dockerfile      # Active - main beambot image
docker/beambot_img/Dockerfile           # Active - lightweight version
docker/bsui/Dockerfile                  # Active - full Bluesky
docker/bsui-minimal/Dockerfile          # Active - minimal Bluesky
docker/ur-driver/Dockerfile             # Active - UR driver
docker/ur-moveit/Dockerfile             # Active - MoveIt
docker/ursim/Dockerfile                 # Active - UR simulator
docker/ur-example/Dockerfile            # Active - example
docker/azure-kinect/Dockerfile.txt      # Deprecated?
.devcontainer/Dockerfile                # Active - VS Code
docker/archive/bsui/Dockerfile          # Archived
docker/archive/erobs-common-img/Dockerfile  # Archived
```

### Shell Scripts (21 total)
```
build.sh
setup.sh
test.sh
start_ursim.sh
docker/img_build.sh
docker/erobs-common-img/start-vnc.sh
docker/beambot_img/start-vnc.sh
docker/ur-driver/entrypoint.sh
docker/ur-moveit/entrypoint.sh
scripts/pdf-launch-scripts/bsui-launch.sh
scripts/pdf-launch-scripts/hello-talker.sh
scripts/pdf-launch-scripts/mtc-moveit-launch.sh
scripts/pdf-launch-scripts/robotiq-driver-entrypoint.sh
scripts/pdf-launch-scripts/robotiq-driver-launch.sh
scripts/pdf-launch-scripts/sample-movement-server-launch.sh
scripts/pdf-launch-scripts/ur-driver-launch.sh
scripts/pdf-launch-scripts/ur-hande-driver-launch.sh
src/custom-ur-descriptions/ur5e_robot_description/urdf/convert_urdf_for_isaac.sh
src/bluesky_ros/archive/local_bsui.sh
.github/actions/lint/run.sh
.github/actions/test/run.sh
```

### CI Workflows (4 total)
```
.github/workflows/ros.yaml
.github/workflows/ruff.yml
.github/workflows/super-linter.yml
.github/workflows/docker-publish.yml
```

---

*End of Review*
