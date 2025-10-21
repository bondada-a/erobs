#!/usr/bin/env python3
"""
Create Letter-sized PDFs with AprilTags at 10mm size
Tag IDs: 2, 3, 4, 5, 6, 7, 8
"""
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
from PIL import Image
import os

def create_tag_pdf(image_path, tag_id, size_mm, output_pdf):
    """Create a Letter-sized PDF with tag at exact size"""

    # Letter size in points (72 points per inch)
    page_width, page_height = letter  # 612 x 792 points

    # Convert mm to points (1 inch = 25.4mm = 72 points)
    size_points = size_mm * 72 / 25.4

    # Create PDF
    c = canvas.Canvas(output_pdf, pagesize=letter)

    # Center the tag on the page
    x = (page_width - size_points) / 2
    y = (page_height - size_points) / 2

    # Add tag image at exact size
    c.drawImage(image_path, x, y, width=size_points, height=size_points,
                preserveAspectRatio=True, mask='auto')

    # Add title (well above the tag)
    c.setFont("Helvetica-Bold", 14)
    title_y = y + size_points + 80
    c.drawCentredString(page_width / 2, title_y,
                        f"AprilTag ID {tag_id} - tag36h11")

    # Add size label
    c.setFont("Helvetica", 12)
    c.drawCentredString(page_width / 2, title_y - 20,
                        f"Size: {size_mm}mm × {size_mm}mm")

    # Add measurement instructions (well below the tag)
    c.setFont("Helvetica", 10)
    instructions_y = y - 80
    c.drawCentredString(page_width / 2, instructions_y,
                        "⚠ IMPORTANT: Print at 100% scale (NO auto-fit)")
    c.drawCentredString(page_width / 2, instructions_y - 15,
                        f"After printing, measure the black square: should be exactly {size_mm}mm")

    # Add cutting guide corners
    c.setStrokeColorRGB(0.8, 0.8, 0.8)
    c.setLineWidth(0.5)
    margin = 5 * mm

    # Top-left corner
    c.line(x - margin, y + size_points, x - margin, y + size_points + margin)
    c.line(x - margin, y + size_points + margin, x, y + size_points + margin)

    # Top-right corner
    c.line(x + size_points + margin, y + size_points,
           x + size_points + margin, y + size_points + margin)
    c.line(x + size_points + margin, y + size_points + margin,
           x + size_points, y + size_points + margin)

    # Bottom-left corner
    c.line(x - margin, y, x - margin, y - margin)
    c.line(x - margin, y - margin, x, y - margin)

    # Bottom-right corner
    c.line(x + size_points + margin, y, x + size_points + margin, y - margin)
    c.line(x + size_points + margin, y - margin, x + size_points, y - margin)

    # Add dimensions (farther from tag)
    c.setFont("Helvetica", 8)
    c.setFillColorRGB(0.5, 0.5, 0.5)
    c.drawString(x + size_points + 20, y + size_points / 2, f"{size_mm}mm")
    c.drawCentredString(x + size_points / 2, y - 35, f"{size_mm}mm")

    # Save PDF
    c.save()
    print(f"✓ Created: {output_pdf}")
    print(f"  Tag ID: {tag_id}, Size: {size_mm}mm × {size_mm}mm")

def main():
    print("=" * 60)
    print("  Creating 10mm AprilTag PDFs (IDs 2-8)")
    print("=" * 60)
    print()

    size_mm = 10
    tag_ids = [2, 3, 4, 5, 6, 7, 8]

    for tag_id in tag_ids:
        image_file = f"tag{tag_id}_highres.png"
        output_file = f"tag{tag_id}_10mm_LETTER.pdf"

        if not os.path.exists(image_file):
            print(f"✗ Error: {image_file} not found")
            continue

        create_tag_pdf(image_file, tag_id, size_mm, output_file)

    print()
    print("=" * 60)
    print("  Print Instructions")
    print("=" * 60)
    print("1. Open the PDF files")
    print("2. Print settings:")
    print("   - Paper: Letter (8.5\" × 11\")")
    print("   - Scale: 100% or 'Actual Size'")
    print("   - NO 'Fit to page' or 'Shrink to fit'")
    print("3. After printing, measure with ruler:")
    print(f"   - Each tag: Black square should be {size_mm}mm (±0.5mm is okay)")
    print("4. Cut with white border (~2mm around black area)")
    print("5. Mount flat on rigid surface")
    print("=" * 60)

if __name__ == '__main__':
    main()
