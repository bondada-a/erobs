# Zivid Camera - Automatic Capture Settings

## Automated Setup (Recommended)

Run one command to automatically configure optimal settings:

```bash
./auto_configure_zivid.sh
```

**What it does:**
1. Runs Capture Assistant to analyze your scene
2. Extracts optimized 2D settings
3. Saves to `src/zivid-ros/cam_settings_2d_auto.yml`
4. Prompts to apply settings immediately
5. Tests capture with new settings

**Options:**
```bash
./auto_configure_zivid.sh        # 60Hz lighting (US/Canada)
./auto_configure_zivid.sh 50hz   # 50Hz lighting (Europe/Asia)
./auto_configure_zivid.sh none   # No ambient light
```

---

## Manual Setup

The Zivid Capture Assistant automatically analyzes your scene and suggests optimal camera settings.

### 1. Run Capture Assistant

```bash
source install/setup.bash

# Run with 2 second max capture time, 60Hz lighting (US/Canada)
ros2 service call /capture_assistant/suggest_settings \
  zivid_interfaces/srv/CaptureAssistantSuggestSettings \
  "{max_capture_time: {sec: 2, nanosec: 0}, ambient_light_frequency: 2}"

# For 50Hz lighting (Europe/Asia), use: ambient_light_frequency: 1
# For no ambient light, use: ambient_light_frequency: 0
```

**Wait ~5-10 seconds** - the camera will capture multiple images to analyze your scene.

### 2. Extract 2D Settings from Response

The response contains full 3D + 2D settings. Extract the `Settings2D` section and save to a file:

**File:** `src/zivid-ros/cam_settings_2d_optimized.yml`

```yaml
__version__: 7
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 2.47           # From assistant output
        Brightness: 2.5
        ExposureTime: 8333       # Optimized for your lighting
        Gain: 1.54
  Processing:
    Color:
      Balance:
        Blue: 1
        Green: 1
        Red: 1
      Experimental:
        Mode: automatic
      Gamma: 1
  Sampling:
    Color: rgb
    Pixel: by2x2
```

### 3. Load Settings

```bash
# Set the camera to use optimized settings
ros2 param set /zivid_camera settings_2d_file_path \
  /path/to/your/cam_settings_2d_optimized.yml
```

### 4. Test Capture

```bash
# Trigger a capture
ros2 service call /capture_2d std_srvs/srv/Trigger "{}"

# View the image
ros2 run rqt_image_view rqt_image_view
# Select /color/image_color from dropdown
```

---

## Manual Settings Adjustment

If you need to adjust settings manually:

**Too dark:** Increase `ExposureTime` or `Brightness`
**Too bright:** Decrease `ExposureTime` or `Brightness`
**Noisy:** Decrease `Gain`, increase `ExposureTime`
**Motion blur:** Decrease `ExposureTime`

---

## Current Setup

**Settings file:** `src/zivid-ros/cam_settings_2d_optimized.yml`
**Capture service:** `/capture_2d`
**Image topic:** `/color/image_color`
**Resolution:** 1224x1024 (by2x2 sampling)

---

## Notes

- Zivid cameras use **triggered capture** (not continuous streaming)
- Settings are optimized for **AprilTag detection** (good contrast, sharp edges)
- Re-run Capture Assistant if lighting conditions change significantly
- For 2D captures, use single acquisition only (multi-acquisition HDR requires same aperture)
