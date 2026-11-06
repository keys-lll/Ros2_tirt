#!/usr/bin/env python3
"""Arrange the competition windows on X11 without sending keys or clicks."""
import sys
import time

import gi

gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
gi.require_version('Wnck', '3.0')
from gi.repository import Gdk, Gtk, Wnck


def main():
    display = Gdk.Display.get_default()
    if display is None or 'X11' not in type(display).__name__:
        print('視窗排列未套用：需要 X11 桌面，請手動選取 mission。', file=sys.stderr)
        return 1
    Wnck.set_client_type(Wnck.ClientType.PAGER)
    screen = Wnck.Screen.get_default()
    if screen is None:
        return 1
    flags = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y |
             Wnck.WindowMoveResizeMask.WIDTH | Wnck.WindowMoveResizeMask.HEIGHT)
    positioned_rviz = set()
    mission_id = None
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        screen.force_update()
        windows = screen.get_windows()
        gui = '--gui' in sys.argv
        mission = next((w for w in windows if w.get_name() == '比賽任務' and
                        (not gui or w.get_class_group() is None or
                         'terminal' not in (w.get_class_group().get_res_class() or '').lower())), None)
        if mission is None:
            if mission_id is not None:
                return 0
            time.sleep(0.2)
            continue
        gdk_window = Gdk.get_default_root_window()
        monitor = display.get_monitor_at_window(gdk_window)
        area = monitor.get_workarea()
        right_width = (min(1100, max(860, int(area.width * 0.65)), area.width - 40)
                       if gui else min(max(420, int(area.width * 0.38)), area.width // 2))
        left_width = area.width - right_width
        focus = mission_id != mission.get_xid()
        if focus:
            mission_id = mission.get_xid()
            mission.unmaximize()
            mission.unminimize(0)
            mission.make_above()
            if gui:
                _, _, width, height = mission.get_geometry()
                mission.set_geometry(Wnck.WindowGravity.STATIC,
                                     Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y,
                                     area.x + (area.width - width) // 2,
                                     area.y + (area.height - height) // 2,
                                     width, height)
            else:
                mission.set_geometry(Wnck.WindowGravity.STATIC, flags,
                                     area.x + left_width, area.y,
                                     right_width, max(200, area.height - 40))
        for window in windows:
            group = window.get_class_group()
            identity = window.get_name().lower()
            if group is not None:
                identity += ' ' + (group.get_res_class() or '').lower()
            if not gui and 'rviz' in identity and window.get_xid() not in positioned_rviz:
                window.unmake_above()
                window.unmaximize()
                window.set_geometry(Wnck.WindowGravity.STATIC, flags,
                                    area.x + (6 if gui else 0), area.y,
                                    area.width - 12 if gui else left_width,
                                    max(200, area.height // 2 - 36 if gui else area.height - 40))
                positioned_rviz.add(window.get_xid())
                focus = True
        if focus:
            mission.activate(0)
            pointer = display.get_default_seat().get_pointer()
            pointer.warp(Gdk.Screen.get_default(),
                         area.x + area.width // 2 if gui else area.x + left_width + right_width // 2,
                         area.y + area.height // 2 if gui else area.y + min(120, area.height // 2))
        if mission_id is not None and (gui or positioned_rviz):
            # Allow the WM to apply asynchronous geometry/focus requests.
            until = time.monotonic() + 0.5
            while time.monotonic() < until:
                while Gtk.events_pending():
                    Gtk.main_iteration_do(False)
                time.sleep(0.02)
            return 0
        time.sleep(0.2)
    if mission_id is None:
        print('找不到「比賽任務」視窗，請手動選取 mission。', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
