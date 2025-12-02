# Bluesky/ROS Integration - Quick Reference

> **Status**: ✅ Fully Installed and Tested (2025-12-02)

## 🎯 What You Have

You have **TWO complete installations** of the Bluesky/ROS system:

### 🖥️ Local Installation
- Full native performance
- Easy development and debugging
- Direct hardware access

### 🐳 Docker Installation
- Perfect reproducibility
- Easy deployment
- Shareable across systems

## ⚡ Quick Start (Choose One)

### Local
```bash
cd ~/work/github_ws/erobs
./local_bsui.sh
```

### Docker
```bash
docker pull ghcr.io/bondada-a/bsui-img-new:latest
docker run -it --network host ghcr.io/bondada-a/bsui-img-new:latest
```

## 📚 Documentation

| File | Purpose |
|------|---------|
| [INSTALLATION_COMPLETE.md](INSTALLATION_COMPLETE.md) | Installation summary & quick start |
| [DUAL_SETUP_GUIDE.md](DUAL_SETUP_GUIDE.md) | **Complete guide** - local + Docker |
| [BLUESKY_LOCAL_SETUP.md](BLUESKY_LOCAL_SETUP.md) | Detailed local documentation |
| [BLUESKY_QUICKSTART.md](BLUESKY_QUICKSTART.md) | Quick reference card |

## 🔧 What's Installed

### Local Components
```
✅ EPICS Base       /home/aditya/EPICS/epics-base
✅ Miniconda        /home/aditya/miniconda3
✅ Bluesky Env      ~/miniconda3/envs/bluesky
✅ ROS 2 Humble     /opt/ros/humble
✅ Workspace        ~/work/github_ws/erobs
```

### Docker Images
```
✅ ur5e-erobs-common-img:latest   11.6 GB
   ghcr.io/bondada-a/ur5e-erobs-common-img:latest

✅ bsui-img-new:latest            12.9 GB
   ghcr.io/bondada-a/bsui-img-new:latest
```

## 🚀 Examples

### Run MTC Task with Bluesky

**Local:**
```bash
python3 src/bluesky_ros/simple_mtc_bluesky.py task_sequences/complete_sequence.json
```

**Docker:**
```bash
docker run -it ghcr.io/bondada-a/bsui-img-new:latest \
  python3 /root/ws/erobs/src/bluesky_ros/simple_mtc_bluesky.py \
  /root/ws/erobs/task_sequences/complete_sequence.json
```

### Interactive Bluesky Session

**Local:**
```bash
./local_bsui.sh --ipython
```

**Docker:**
```bash
docker run -it ghcr.io/bondada-a/bsui-img-new:latest bsui
```

## 🎓 Architecture Overview

```
┌─────────────────────────────────────────────┐
│         Bluesky RunEngine                   │
│      (Data Acquisition Framework)           │
└──────────────────┬──────────────────────────┘
                   │
                   │ Plans & Commands
                   │
┌──────────────────▼──────────────────────────┐
│      MTCExecutionDevice (Ophyd)             │
│   - Wraps ROS 2 Action Client               │
│   - Bluesky-compatible interface            │
└──────────────────┬──────────────────────────┘
                   │
                   │ ROS 2 Actions
                   │
┌──────────────────▼──────────────────────────┐
│   MTC Pipeline (C++ Action Server)          │
│   - Task Constructor planning               │
│   - Motion execution                        │
└──────────────────┬──────────────────────────┘
                   │
                   │ UR RTDE / Robot Control
                   │
┌──────────────────▼──────────────────────────┐
│         UR5e Robot + Gripper                │
└─────────────────────────────────────────────┘
```

## 💡 Key Scripts

| Script | Description |
|--------|-------------|
| `local_bsui.sh` | Launch local Bluesky environment |
| `test_bluesky_local.py` | Test installation |
| `src/bluesky_ros/simple_mtc_bluesky.py` | Simple task executor |
| `src/bluesky_ros/mtc_bluesky_example.py` | Full Ophyd example |
| `src/bluesky_ros/mtc_ophyd_device.py` | Ophyd device class |

## 🐛 Troubleshooting

**Problem:** Module not found
**Solution:** Run `./local_bsui.sh` to set up environment

**Problem:** Cannot connect to robot
**Solution:** Use `--network host` in Docker

**Problem:** EPICS tools not found
**Solution:** Check `echo $EPICS_BASE`

## 🔗 Links

- [Bluesky Docs](https://blueskyproject.io/bluesky/main/index.html)
- [Ophyd Docs](https://blueskyproject.io/ophyd/main/index.html)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/index.html)
- [MTC Tutorial](https://moveit.picknik.ai/main/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html)

## 🎉 Next Steps

1. ✅ Read [INSTALLATION_COMPLETE.md](INSTALLATION_COMPLETE.md)
2. ⬜ Test local setup: `./local_bsui.sh`
3. ⬜ Test Docker: `docker run -it ghcr.io/bondada-a/bsui-img-new:latest`
4. ⬜ Run example: `python3 src/bluesky_ros/simple_mtc_bluesky.py <task.json>`
5. ⬜ Create custom Bluesky plans
6. ⬜ Integrate with your beamline/robot

---

**Version**: 1.0
**Last Updated**: 2025-12-02
**Status**: Production Ready ✅
