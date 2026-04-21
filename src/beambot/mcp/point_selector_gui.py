#!/usr/bin/env python3
"""Standalone tkinter point selector GUI for the get_point_3d MCP tool.

Launched as a subprocess by beambot_mcp_server.py. No ROS or OpenCV dependencies
for the GUI itself — uses tkinter (always available) + PIL for image display.

Usage:
    python3 point_selector_gui.py <image_path> [--x 500] [--y 300] [--title "Point Selector"]

Output (single JSON line on stdout):
    Confirmed: {"pixel_x": 485, "pixel_y": 412, "confirmed": true}
    Cancelled: {"confirmed": false}
"""

import argparse
import json
import os
import subprocess as _sp
import sys
import tkinter as tk

from PIL import Image, ImageDraw, ImageTk


def _ensure_display():
    """Ensure DISPLAY and XAUTHORITY are set for X11 GUI access."""
    if not os.environ.get("DISPLAY"):
        try:
            result = _sp.run(
                ["bash", "-c",
                 "cat /proc/$(pgrep -u $USER -x gnome-shell || "
                 "pgrep -u $USER -x Xwayland || echo 1)/environ "
                 "2>/dev/null | tr '\\0' '\\n' | grep ^DISPLAY= | "
                 "head -1 | cut -d= -f2"],
                capture_output=True, text=True, timeout=3,
            )
            os.environ["DISPLAY"] = result.stdout.strip() or ":1"
        except Exception:
            os.environ["DISPLAY"] = ":1"

    if not os.environ.get("XAUTHORITY"):
        for candidate in [
            f"/run/user/{os.getuid()}/gdm/Xauthority",
            os.path.expanduser("~/.Xauthority"),
        ]:
            if os.path.exists(candidate):
                os.environ["XAUTHORITY"] = candidate
                break


_ensure_display()


class PointSelector:
    """Tkinter-based point selector with image display and crosshair."""

    CROSSHAIR_SIZE = 20
    CROSSHAIR_COLOR = "red"
    MAX_WINDOW_W = 1280
    MAX_WINDOW_H = 800

    def __init__(
        self,
        image_path: str,
        initial_point: tuple[int, int] | None = None,
        title: str = "Point Selector",
    ):
        self.pil_image = Image.open(image_path)
        self.img_w, self.img_h = self.pil_image.size
        self.title = title
        # Point in original image coordinates
        self.current_point = initial_point
        self.result: Dict | None = None

        # Compute display scale to fit window
        self.scale = min(
            self.MAX_WINDOW_W / self.img_w,
            self.MAX_WINDOW_H / self.img_h,
            1.0,  # Don't upscale
        )
        self.disp_w = int(self.img_w * self.scale)
        self.disp_h = int(self.img_h * self.scale)

        # Pre-scale the base image for display
        self._base_display = self.pil_image.resize(
            (self.disp_w, self.disp_h), Image.LANCZOS,
        )

    def _draw_crosshair(self) -> ImageTk.PhotoImage:
        """Return a PhotoImage with crosshair drawn at current_point."""
        img = self._base_display.copy()
        if self.current_point is not None:
            draw = ImageDraw.Draw(img)
            # Convert original coords to display coords
            dx = int(self.current_point[0] * self.scale)
            dy = int(self.current_point[1] * self.scale)
            s = self.CROSSHAIR_SIZE

            # Crosshair lines
            draw.line([(dx - s, dy), (dx + s, dy)], fill="red", width=2)
            draw.line([(dx, dy - s), (dx, dy + s)], fill="red", width=2)

            # Coordinate label
            label = f"({self.current_point[0]}, {self.current_point[1]})"
            draw.text((dx + 12, dy - 18), label, fill="white")
            draw.text((dx + 11, dy - 19), label, fill="black")

        return ImageTk.PhotoImage(img)

    def run(self) -> Dict:
        """Run the tkinter event loop. Returns result dict."""
        root = tk.Tk()
        root.title(self.title)
        root.configure(bg="#282828")

        # Banner
        banner = tk.Label(
            root,
            text="Click to select point.  Enter/Space = confirm,  Esc = cancel",
            bg="#282828",
            fg="#cccccc",
            font=("monospace", 11),
            pady=8,
        )
        banner.pack(fill=tk.X)

        # Canvas for image
        canvas = tk.Canvas(
            root, width=self.disp_w, height=self.disp_h,
            highlightthickness=0, bg="black",
        )
        canvas.pack()

        # Initial render
        self._tk_photo = self._draw_crosshair()
        canvas_image = canvas.create_image(0, 0, anchor=tk.NW, image=self._tk_photo)

        def _update_display():
            self._tk_photo = self._draw_crosshair()
            canvas.itemconfig(canvas_image, image=self._tk_photo)

        def _on_click(event):
            # Convert display coords back to original image coords
            orig_x = int(event.x / self.scale)
            orig_y = int(event.y / self.scale)
            # Clamp to image bounds
            orig_x = max(0, min(orig_x, self.img_w - 1))
            orig_y = max(0, min(orig_y, self.img_h - 1))
            self.current_point = (orig_x, orig_y)
            _update_display()

        def _on_confirm(event=None):
            if self.current_point is not None:
                self.result = {
                    "pixel_x": self.current_point[0],
                    "pixel_y": self.current_point[1],
                    "confirmed": True,
                }
                root.destroy()

        def _on_cancel(event=None):
            self.result = {"confirmed": False}
            root.destroy()

        canvas.bind("<Button-1>", _on_click)
        root.bind("<Return>", _on_confirm)
        root.bind("<space>", _on_confirm)
        root.bind("<Escape>", _on_cancel)
        root.protocol("WM_DELETE_WINDOW", _on_cancel)

        # Center window on screen
        root.update_idletasks()
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        x = (sw - self.disp_w) // 2
        y = (sh - self.disp_h) // 2
        root.geometry(f"+{x}+{y}")

        root.mainloop()
        return self.result or {"confirmed": False}


def main():
    parser = argparse.ArgumentParser(description="Point selector GUI")
    parser.add_argument("image_path", help="Path to the image file")
    parser.add_argument("--x", type=int, default=None, help="Initial X coordinate")
    parser.add_argument("--y", type=int, default=None, help="Initial Y coordinate")
    parser.add_argument("--title", default="Point Selector", help="Window title")
    args = parser.parse_args()

    initial_point = None
    if args.x is not None and args.y is not None:
        initial_point = (args.x, args.y)

    try:
        selector = PointSelector(args.image_path, initial_point, args.title)
        result = selector.run()
    except Exception as e:
        result = {"confirmed": False, "error": str(e)}
        print(json.dumps(result))
        sys.exit(1)

    print(json.dumps(result))


if __name__ == "__main__":
    main()
