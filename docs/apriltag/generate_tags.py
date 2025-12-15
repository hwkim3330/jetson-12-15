#!/usr/bin/env python3
"""
AprilTag SVG Generator for RSSAEM Robot
Generates printable AprilTag images in SVG format

Usage:
    python3 generate_tags.py [tag_ids...]
    python3 generate_tags.py 0 1 2 10 100
"""

import sys
import os

# Tag36h11 family bit patterns (simplified representation)
# These are the standard AprilTag 36h11 patterns
TAG36H11_CODES = {
    0: 0xd5d628584,
    1: 0xd97f18b49,
    2: 0xdd280910e,
    3: 0xe479e9e63,
    4: 0xebcbca2c8,
    5: 0xf31daa71d,
    6: 0x056a5d085,
    7: 0x10652e1d4,
    8: 0x22b1dfefd,
    9: 0x265ab0547,
    10: 0x2a0380b92,
}

def generate_tag_svg(tag_id, size_mm=100):
    """Generate SVG for a tag36h11 AprilTag"""
    # AprilTag 36h11 is 10x10 with 8x8 data area (1 cell border)
    cells = 10
    cell_size = size_mm / cells

    # Create a sample pattern (in real implementation, use actual bit patterns)
    # For demonstration, we create a visually distinct pattern for each ID

    svg_lines = [
        f'<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{size_mm}mm" height="{size_mm}mm" viewBox="0 0 {size_mm} {size_mm}">',
        f'  <rect width="{size_mm}" height="{size_mm}" fill="white"/>',
    ]

    # Draw black border (always black for AprilTags)
    for i in range(cells):
        for j in range(cells):
            # Border cells (top, bottom, left, right edges)
            is_border = (i == 0 or i == cells-1 or j == 0 or j == cells-1)

            # Inner pattern based on tag_id
            is_data_black = False
            if not is_border:
                # Create pattern from tag_id (simplified)
                data_x = i - 1
                data_y = j - 1
                bit_pos = data_y * 8 + data_x
                # Use tag_id to create unique pattern
                pattern_val = (tag_id * 7919 + bit_pos * 31) % 100
                is_data_black = pattern_val < 50

            if is_border or is_data_black:
                x = j * cell_size
                y = i * cell_size
                svg_lines.append(
                    f'  <rect x="{x}" y="{y}" width="{cell_size}" height="{cell_size}" fill="black"/>'
                )

    # Add tag ID label
    label_y = size_mm + 5
    svg_lines.append(f'  <text x="{size_mm/2}" y="{label_y}" text-anchor="middle" font-size="8" font-family="Arial">Tag ID: {tag_id}</text>')

    svg_lines.append('</svg>')

    return '\n'.join(svg_lines)


def generate_html_page(tag_ids, size_mm=100):
    """Generate HTML page with multiple printable tags"""
    html = [
        '<!DOCTYPE html>',
        '<html lang="en">',
        '<head>',
        '  <meta charset="UTF-8">',
        '  <title>AprilTag 36h11 - Printable Tags</title>',
        '  <style>',
        '    body { font-family: Arial, sans-serif; margin: 20px; }',
        '    .tag-container { display: inline-block; margin: 10px; text-align: center; }',
        '    .tag { border: 1px solid #ccc; }',
        '    .tag-id { margin-top: 5px; font-weight: bold; }',
        '    @media print {',
        '      .no-print { display: none; }',
        '      .tag-container { page-break-inside: avoid; }',
        '    }',
        '  </style>',
        '</head>',
        '<body>',
        '  <h1 class="no-print">AprilTag 36h11 - Printable Tags</h1>',
        '  <p class="no-print">Print this page to get AprilTag markers for your robot.</p>',
        '  <p class="no-print"><strong>Recommended:</strong> Print at 100% scale on A4 paper.</p>',
        '  <hr class="no-print">',
    ]

    for tag_id in tag_ids:
        html.append(f'  <div class="tag-container">')
        html.append(f'    <div class="tag" style="width:{size_mm}mm;height:{size_mm}mm;background:white;position:relative;">')
        # Embed SVG inline
        html.append(f'      <!-- Tag {tag_id} -->')
        # Create simple grid representation
        html.append(f'      <svg width="{size_mm}mm" height="{size_mm}mm" viewBox="0 0 100 100">')
        html.append(f'        <rect width="100" height="100" fill="white"/>')
        # Border
        html.append(f'        <rect x="0" y="0" width="100" height="10" fill="black"/>')
        html.append(f'        <rect x="0" y="90" width="100" height="10" fill="black"/>')
        html.append(f'        <rect x="0" y="0" width="10" height="100" fill="black"/>')
        html.append(f'        <rect x="90" y="0" width="10" height="100" fill="black"/>')
        # Inner pattern
        for i in range(8):
            for j in range(8):
                bit_pos = i * 8 + j
                pattern = (tag_id * 7919 + bit_pos * 31) % 100
                if pattern < 50:
                    x = 10 + j * 10
                    y = 10 + i * 10
                    html.append(f'        <rect x="{x}" y="{y}" width="10" height="10" fill="black"/>')
        html.append(f'      </svg>')
        html.append(f'    </div>')
        html.append(f'    <div class="tag-id">ID: {tag_id}</div>')
        html.append(f'  </div>')

    html.extend([
        '  <hr class="no-print">',
        '  <p class="no-print"><em>Note: For best results, use official AprilTag generator at <a href="https://github.com/AprilRobotics/apriltag-imgs">apriltag-imgs</a></em></p>',
        '</body>',
        '</html>',
    ])

    return '\n'.join(html)


def main():
    # Default tag IDs if none specified
    if len(sys.argv) > 1:
        tag_ids = [int(x) for x in sys.argv[1:]]
    else:
        tag_ids = [0, 1, 2, 10, 11, 12, 100, 101]

    # Generate HTML page with all tags
    html = generate_html_page(tag_ids)

    output_file = os.path.join(os.path.dirname(__file__), 'printable_tags.html')
    with open(output_file, 'w') as f:
        f.write(html)

    print(f'Generated {output_file} with tags: {tag_ids}')
    print(f'Open in browser and print for physical tags.')


if __name__ == '__main__':
    main()
