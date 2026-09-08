#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Show a non-modal popup when J2-J3 coupling is violated (slider / Randomize).

ROS2 port of Option 1 coupling_warn_gui.
Subscribes to /ultraarm_p1/j2_j3_coupling_warning from joint_coupling_node.
- Invalid: open/update window with latest J2-J3 text
- ok: auto-close the window
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    import Tkinter as tk
    import ttk


class CouplingWarnGui(Node):
    def __init__(self):
        super().__init__("coupling_warn_gui")
        self._lock = threading.Lock()
        self._latest_text = None
        self._should_close = False
        self._win = None
        self._label = None

        self.root = tk.Tk()
        self.root.withdraw()
        self.root.title("ultraArm P1 Coupling")

        self.create_subscription(
            String,
            "/ultraarm_p1/j2_j3_coupling_warning",
            self._on_warning,
            10,
        )
        self.root.after(100, self._poll)

    def _on_warning(self, msg):
        data = (msg.data or "").strip()
        with self._lock:
            if data == "ok" or data.startswith("ok"):
                self._should_close = True
                self._latest_text = None
                return
            self._should_close = False
            self._latest_text = data

    def _ensure_window(self):
        if self._win is not None and self._win.winfo_exists():
            return
        win = tk.Toplevel(self.root)
        # Hide until geometry is centered to avoid a top-left flash.
        win.withdraw()
        win.title("J2-J3 Coupling Limit")
        win.attributes("-topmost", True)
        win.resizable(False, False)
        frm = ttk.Frame(win, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            frm,
            text=(
                "J2-J3 combination is out of the allowed coupled range.\n"
                "The RViz model will stay at the last valid pose."
            ),
            justify="left",
        ).grid(row=0, column=0, sticky="w")
        self._label = ttk.Label(frm, text="", justify="left")
        self._label.grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Button(frm, text="OK", command=self._on_user_close).grid(
            row=2, column=0, sticky="e", pady=(12, 0)
        )
        win.protocol("WM_DELETE_WINDOW", self._on_user_close)
        self._win = win

    def _center_window(self, win):
        win.update_idletasks()
        w = max(win.winfo_reqwidth(), 1)
        h = max(win.winfo_reqheight(), 1)
        x = max((win.winfo_screenwidth() - w) // 2, 0)
        y = max((win.winfo_screenheight() - h) // 2, 0)
        win.geometry("%dx%d+%d+%d" % (w, h, x, y))

    def _show_centered(self):
        if self._win is None:
            return
        try:
            self._center_window(self._win)
            self._win.deiconify()
            self._win.lift()
        except Exception:
            pass

    def _on_user_close(self):
        self._destroy_window()

    def _destroy_window(self):
        if self._win is not None:
            try:
                if self._win.winfo_exists():
                    self._win.destroy()
            except Exception:
                pass
        self._win = None
        self._label = None

    def _poll(self):
        text = None
        should_close = False
        with self._lock:
            if self._should_close:
                should_close = True
                self._should_close = False
            if self._latest_text is not None:
                text = self._latest_text
                self._latest_text = None

        if should_close:
            self._destroy_window()

        if text is not None:
            self._ensure_window()
            if self._label is not None:
                self._label.configure(text=text)
            self._show_centered()

        if not rclpy.ok():
            self._destroy_window()
            try:
                self.root.quit()
            except Exception:
                pass
            return
        self.root.after(100, self._poll)

    def spin_gui(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CouplingWarnGui()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.spin_gui()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
