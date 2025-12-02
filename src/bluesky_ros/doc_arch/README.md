# Bluesky/ROS Integration Documentation

> **Last Updated**: 2025-12-02
> **Status**: Production Ready ✅

This documentation covers the Bluesky integration for MoveIt Task Constructor (MTC), enabling data acquisition orchestration with robot manipulation tasks.

---

## 📖 Documentation Structure

This documentation is organized by user journey:

```
doc_arch/
├── quickstart/          → New users start here
├── guides/              → In-depth usage patterns
├── reference/           → Technical details & setup
└── experimental/        → Advanced features (TaskBuilder)
```

---

## 🚀 Quick Navigation

### New to Bluesky/ROS Integration?
**Start with these in order:**

1. **[quickstart/README_BLUESKY.md](quickstart/README_BLUESKY.md)** - Overview & basic concepts (5 min read)
2. **[quickstart/GETTING_STARTED.md](quickstart/GETTING_STARTED.md)** - First steps & hello world
3. **[quickstart/HOW_TO_USE.md](quickstart/HOW_TO_USE.md)** - Common usage patterns

### Setting Up Your Environment?

- **[reference/INSTALLATION_COMPLETE.md](reference/INSTALLATION_COMPLETE.md)** - What's been installed
- **[reference/BLUESKY_LOCAL_SETUP.md](reference/BLUESKY_LOCAL_SETUP.md)** - Complete local setup guide
- **[reference/DOCKER_IMAGES_STATUS.md](reference/DOCKER_IMAGES_STATUS.md)** - Docker deployment info
- **[guides/DUAL_SETUP_GUIDE.md](guides/DUAL_SETUP_GUIDE.md)** - Local vs Docker comparison

### Ready to Use Interactively?

- **[guides/INTERACTIVE_BLUESKY_GUIDE.md](guides/INTERACTIVE_BLUESKY_GUIDE.md)** - Interactive workflow patterns
- **[guides/ASYNC_DEVICE_GUIDE.md](guides/ASYNC_DEVICE_GUIDE.md)** - Non-blocking execution & cancellation

### Want Advanced Features?

- **[experimental/TASK_BUILDER_QUICKSTART.md](experimental/TASK_BUILDER_QUICKSTART.md)** - Programmatic task creation
- **[experimental/LOAD_IN_BSUI.md](experimental/LOAD_IN_BSUI.md)** - Loading in beamline BSUI environment

### Need a Command Reference?

- **[reference/BLUESKY_QUICKSTART.md](reference/BLUESKY_QUICKSTART.md)** - Common commands cheat sheet

---

## 📁 Core Files Reference

After cleanup (2025-12-02), here are the active files:

### Core Implementation (`src/bluesky_ros/`)

| File | Purpose | Status |
|------|---------|--------|
| **mtc_ophyd_device_async.py** | Async MTC device (recommended) | ✅ Active |
| **mtc_ophyd_device.py** | Original blocking device (legacy) | ⚠️ Legacy |
| **simple_mtc_bluesky.py** | Command-line task executor | ✅ Active |
| **task_builder.py** | Programmatic task construction | 🧪 Experimental |

### Interactive Shells (`src/bluesky_ros/`)

| File | Purpose | When to Use |
|------|---------|-------------|
| **bluesky_shell_basic.py** | Basic interactive setup | Quick testing, matches original workflow |
| **bluesky_shell_taskbuilder.py** | Interactive shell with TaskBuilder | Programmatic task creation |

### Test Scripts (`src/bluesky_ros/tests/`)

| File | Purpose |
|------|---------|
| **test_bluesky_local.py** | Verify local installation |
| **test_async_device.py** | Test async device functionality |

### Example Scripts (`src/bluesky_ros/doc_arch/quickstart/` & `experimental/`)

| File | Location | Purpose |
|------|----------|---------|
| **simple_example.py** | quickstart/ | Basic usage example |
| **example_task_builder.py** | experimental/ | TaskBuilder example |
| **load_builder.py** | experimental/ | Load TaskBuilder in IPython |
| **load_robot.py** | experimental/ | Load robot device in IPython |

---

## 🎯 Common Use Cases → Documentation Map

| I want to... | Read this |
|--------------|-----------|
| **Understand what this is** | [quickstart/README_BLUESKY.md](quickstart/README_BLUESKY.md) |
| **Get started quickly** | [quickstart/GETTING_STARTED.md](quickstart/GETTING_STARTED.md) |
| **Use it interactively** | [guides/INTERACTIVE_BLUESKY_GUIDE.md](guides/INTERACTIVE_BLUESKY_GUIDE.md) |
| **Run tasks without blocking** | [guides/ASYNC_DEVICE_GUIDE.md](guides/ASYNC_DEVICE_GUIDE.md) |
| **Set up local environment** | [reference/BLUESKY_LOCAL_SETUP.md](reference/BLUESKY_LOCAL_SETUP.md) |
| **Use Docker instead** | [guides/DUAL_SETUP_GUIDE.md](guides/DUAL_SETUP_GUIDE.md) |
| **Build tasks programmatically** | [experimental/TASK_BUILDER_QUICKSTART.md](experimental/TASK_BUILDER_QUICKSTART.md) |
| **See command examples** | [reference/BLUESKY_QUICKSTART.md](reference/BLUESKY_QUICKSTART.md) |
| **Debug installation issues** | [reference/INSTALLATION_COMPLETE.md](reference/INSTALLATION_COMPLETE.md) |

---

## 🎓 Recommended Learning Path

### For Beginners:
```
1. quickstart/README_BLUESKY.md        (understand concepts)
2. quickstart/GETTING_STARTED.md        (first code)
3. quickstart/HOW_TO_USE.md             (common patterns)
4. Run: python3 bluesky_shell_basic.py  (try it!)
```

### For Interactive Users:
```
1. guides/INTERACTIVE_BLUESKY_GUIDE.md  (workflow patterns)
2. Run: python3 bluesky_shell_basic.py  (basic shell)
3. guides/ASYNC_DEVICE_GUIDE.md         (non-blocking execution)
4. Try: Example 3 from ASYNC_DEVICE_GUIDE (cancellation)
```

### For Advanced Users:
```
1. guides/ASYNC_DEVICE_GUIDE.md                  (async features)
2. experimental/TASK_BUILDER_QUICKSTART.md       (programmatic tasks)
3. Read: mtc_ophyd_device_async.py source        (implementation)
4. Create custom plans with task_builder.py
```

---

## ⚡ Quick Start Commands

```bash
# Test your installation
cd ~/work/github_ws/erobs
python3 src/bluesky_ros/tests/test_bluesky_local.py

# Start interactive shell (basic)
python3 src/bluesky_ros/bluesky_shell_basic.py

# Start interactive shell (with TaskBuilder)
python3 src/bluesky_ros/bluesky_shell_taskbuilder.py

# Run a single task from command line
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json

# Test async device
python3 src/bluesky_ros/tests/test_async_device.py
```

---

## 🔧 Key Architectural Decisions

### Why Two Devices?

**MTCExecutionDevice (Original)**
- ✅ Simple, straightforward
- ❌ `wait=False` doesn't work (always blocks)
- 📖 See: [guides/INTERACTIVE_BLUESKY_GUIDE.md](guides/INTERACTIVE_BLUESKY_GUIDE.md)

**MTCExecutionDeviceAsync (Recommended)**
- ✅ True async execution
- ✅ Proper cancellation support
- ✅ Background task execution
- 📖 See: [guides/ASYNC_DEVICE_GUIDE.md](guides/ASYNC_DEVICE_GUIDE.md)

### Why TaskBuilder?

TaskBuilder allows programmatic task creation instead of editing JSON files:

```python
# Without TaskBuilder (manual JSON)
# Edit task_sequences/my_task.json by hand

# With TaskBuilder
json_file = builder.pick_sequence('approach', 'grasp', 'retreat')
RE(bps.abs_set(mtc, {'json_file': json_file, 'robot_ip': ROBOT_IP}))
```

📖 See: [experimental/TASK_BUILDER_QUICKSTART.md](experimental/TASK_BUILDER_QUICKSTART.md)

---

## 🧹 Recent Cleanup (2025-12-02)

**Removed Unused Files:**
- ❌ `ophyd_ros.py` - Unused base class
- ❌ `mtc_bluesky_example.py` - Outdated example
- ❌ `unused/` directory - Old deprecated code

**Renamed for Clarity:**
- ✅ `interactive_bluesky.py` → `bluesky_shell_taskbuilder.py`
- ✅ `quick_bluesky_interactive.py` → `bluesky_shell_basic.py`

**Organized Documentation:**
- ✅ Created folder structure (quickstart/, guides/, reference/, experimental/)
- ✅ Moved all docs to appropriate locations
- ✅ Moved tests to `tests/` folder
- ✅ Created this master index

---

## 🐛 Troubleshooting Quick Links

| Problem | Solution |
|---------|----------|
| Import errors | [reference/INSTALLATION_COMPLETE.md](reference/INSTALLATION_COMPLETE.md) |
| Task won't cancel | [guides/ASYNC_DEVICE_GUIDE.md](guides/ASYNC_DEVICE_GUIDE.md) § Cancellation |
| wait=False still blocks | Use `MTCExecutionDeviceAsync` - see [guides/ASYNC_DEVICE_GUIDE.md](guides/ASYNC_DEVICE_GUIDE.md) |
| Docker can't see local ROS | [guides/DUAL_SETUP_GUIDE.md](guides/DUAL_SETUP_GUIDE.md) § ROS_DOMAIN_ID |
| Python version conflicts | [reference/DOCKER_IMAGES_STATUS.md](reference/DOCKER_IMAGES_STATUS.md) |

---

## 📚 External Resources

- [Bluesky Project](https://blueskyproject.io/bluesky/main/index.html)
- [Ophyd Documentation](https://blueskyproject.io/ophyd/main/index.html)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/index.html)
- [MoveIt Task Constructor Tutorial](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html)

---

## 🎉 Quick Validation

Run these to verify everything works:

```bash
# 1. Check imports
python3 -c "from bluesky_ros.mtc_ophyd_device_async import MTCExecutionDeviceAsync; print('✅ Imports OK')"

# 2. Run test suite
python3 src/bluesky_ros/tests/test_bluesky_local.py

# 3. Try interactive shell
python3 src/bluesky_ros/bluesky_shell_basic.py
# Type: robot (should show device info)
# Ctrl+D to exit
```

---

**Created**: 2025-12-02
**Last Cleanup**: 2025-12-02
**Status**: Production Ready ✅
**Next Review**: When adding new features
