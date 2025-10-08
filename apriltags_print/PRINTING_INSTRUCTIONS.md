# AprilTag Printing Instructions - 5mm and 10mm Tags

## Downloaded Files

- `tag36_11_00000.png` → Will print as **5mm** tag (ID 0)
- `tag36_11_00001.png` → Will print as **10mm** tag (ID 1)

---

## Option 1: Print with ImageMagick (Most Accurate)

```bash
cd apriltags_print/

# Create 5mm tag PDF (300 DPI)
convert tag36_11_00000.png -resize 59x59 -density 300 -units PixelsPerInch tag0_5mm.pdf

# Create 10mm tag PDF (300 DPI)
convert tag36_11_00001.png -resize 118x118 -density 300 -units PixelsPerInch tag1_10mm.pdf

# Print these PDFs at 100% scale
```

**Calculation:** At 300 DPI:
- 5mm = 59 pixels (5mm ÷ 25.4mm/inch × 300 DPI)
- 10mm = 118 pixels (10mm ÷ 25.4mm/inch × 300 DPI)

---

## Option 2: Print from Browser (Simple)

1. **Open image in browser:**
   ```bash
   firefox tag36_11_00000.png &
   ```

2. **Print settings:**
   - Scale: **Custom** → Enter exact size
   - For tag 0: Width = **5mm**, Height = **5mm**
   - For tag 1: Width = **10mm**, Height = **10mm**
   - Quality: **High/Best**

3. **IMPORTANT:** Disable "Fit to page" and "Scale to fit"

---

## Option 3: LibreOffice (Most Control)

```bash
cd apriltags_print/
./create_printable_doc.sh
```

Opens LibreOffice with tags at exact sizes. File → Print.

---

## After Printing - CRITICAL STEPS

### 1. **Measure with Ruler/Caliper**
- Tag 0: Black square should be **exactly 5.0mm** ±0.2mm
- Tag 1: Black square should be **exactly 10.0mm** ±0.2mm
- Measure the **black area only** (not white border)

### 2. **Cut Carefully**
- Include white border (~2mm around black area)
- Keep edges straight
- Don't wrinkle or fold

### 3. **Mount Flat**
- Tape to rigid surface (cardboard, plastic, wood)
- Ensure tag is completely flat
- No shadows or wrinkles

---

## Detection Challenges - Small Tags

**5mm tags:**
- ⚠️ **Very challenging** to detect
- Optimal distance: **5-15cm** from camera
- Requires:
  - Excellent lighting (no shadows)
  - Sharp camera focus
  - Stable mounting
  - Clean print quality

**10mm tags:**
- ✓ **Easier** than 5mm
- Optimal distance: **10-30cm** from camera
- Still needs good lighting and focus

**Tips:**
- Start with 10mm tag first (easier)
- Use bright, diffuse lighting
- Keep tag perpendicular to camera
- Avoid glare on white areas

---

## Update Configuration

After printing, update tag sizes in config:

**File:** `src/mtc_pipeline/config/apriltag_config.yaml`

```yaml
tag_sizes:
  0: 0.005   # 5mm tag
  1: 0.010   # 10mm tag
```

Then rebuild:
```bash
colcon build --packages-select mtc_pipeline
```

---

## Quick Test Commands

```bash
# Trigger capture
ros2 service call /capture_2d std_srvs/srv/Trigger "{}"

# Check detections
ros2 topic echo /apriltag/detections --once

# Check TF for tag 0
ros2 run tf2_ros tf2_echo base_link tag36h11:0

# Check TF for tag 1
ros2 run tf2_ros tf2_echo base_link tag36h11:1
```
