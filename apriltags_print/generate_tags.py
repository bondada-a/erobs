#!/usr/bin/env python3
"""
Generate AprilTag printable PDFs with exact sizes
"""
import subprocess
import sys
import os

def generate_tag_svg(tag_id, size_mm, output_file):
    """Generate AprilTag SVG using apriltag-gen"""
    # SVG units are in mm for easy printing

    svg_content = f'''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg width="{size_mm}mm" height="{size_mm}mm" viewBox="0 0 {size_mm} {size_mm}"
     xmlns="http://www.w3.org/2000/svg">
  <!-- AprilTag ID {tag_id} - {size_mm}mm (tag36h11) -->
  <rect width="{size_mm}" height="{size_mm}" fill="white"/>

  <!-- This is a placeholder - use actual tag image -->
  <text x="{size_mm/2}" y="{size_mm/2}" text-anchor="middle"
        font-size="2" fill="black">
    Tag {tag_id}
    {size_mm}mm
  </text>

  <text x="{size_mm/2}" y="{size_mm-1}" text-anchor="middle"
        font-size="1" fill="red">
    Download actual tag from:
    github.com/AprilRobotics/apriltag-imgs
  </text>
</svg>'''

    with open(output_file, 'w') as f:
        f.write(svg_content)

    print(f"Created: {output_file}")

def create_print_instructions(tag_sizes):
    """Create printing instructions"""
    instructions = f'''# AprilTag Printing Instructions

## Tags to Print

{"".join([f"- **Tag ID {i}**: {size}mm x {size}mm\\n" for i, size in enumerate(tag_sizes)])}

## Download Actual Tag Images

**IMPORTANT**: The SVG files are placeholders. Download actual AprilTag images:

1. Go to: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
2. Download tags: `tag36_11_00000.png`, `tag36_11_00001.png`, etc.
3. Or use this command:

```bash
cd apriltags_print/

# Download tag 0
wget https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png

# Download tag 1
wget https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00001.png
```

## Print at Exact Size

### Method 1: Using ImageMagick (Recommended)

```bash
# For 5mm tag (ID 0)
convert tag36_11_00000.png -resize 5x5mm\\! -density 300 tag0_5mm.pdf

# For 10mm tag (ID 1)
convert tag36_11_00001.png -resize 10x10mm\\! -density 300 tag1_10mm.pdf
```

### Method 2: Manual Print Settings

1. Open PNG in image viewer
2. Print settings:
   - **Scale**: 100% (no auto-fit)
   - **Page size**: A4
   - **Print actual size**: ENABLED
3. Measure printed tag with ruler - should be exactly {tag_sizes[0]}mm and {tag_sizes[1]}mm

## Print Settings
- **Paper**: White paper (good contrast)
- **Quality**: High/Best
- **Color**: Black & White is fine
- **Margins**: Small or none

## After Printing

1. **Cut tags precisely** - include white border
2. **Measure with ruler** - verify exact size
3. **Mount on flat surface** - keep tag flat (no wrinkles)
4. **Good lighting** - avoid shadows/glare

## Detection Tips for Small Tags

**5mm tags:**
- Detection range: ~5-15cm from camera
- Needs very good lighting
- Camera must be in focus at close range
- May need to reduce camera's field of view

**10mm tags:**
- Detection range: ~10-30cm from camera
- Easier to detect than 5mm
- Still needs good lighting

## Verify Size

After printing, measure the **black area** (not including white border):
- Tag 0: Should be exactly {tag_sizes[0]}mm
- Tag 1: Should be exactly {tag_sizes[1]}mm

If size is wrong, your printer may have scaled the image!
'''

    return instructions

def main():
    tag_sizes = [5, 10]  # mm

    print("="*60)
    print("  AprilTag Printable Generator")
    print("="*60)

    # Generate placeholder SVGs
    for i, size in enumerate(tag_sizes):
        output = f"apriltags_print/tag{i}_{size}mm_placeholder.svg"
        generate_tag_svg(i, size, output)

    # Create instructions
    instructions_file = "apriltags_print/PRINTING_INSTRUCTIONS.md"
    with open(instructions_file, 'w') as f:
        f.write(create_print_instructions(tag_sizes))

    print(f"\\nCreated: {instructions_file}")

    print("\\n" + "="*60)
    print("  Next Steps:")
    print("="*60)
    print("1. Download actual tag images (see PRINTING_INSTRUCTIONS.md)")
    print("2. Use ImageMagick to create exact-size PDFs")
    print("3. Print and verify size with ruler")
    print("="*60)

if __name__ == '__main__':
    main()
