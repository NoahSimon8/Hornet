# tools/live_plot.py
import argparse
import ctypes
import logging
import math
import os
import queue
import sys
import threading
import time
from collections import deque


def _setup_logger(log_path):
    logger = logging.getLogger("live_plot")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    logger.propagate = False
    fmt = logging.Formatter("[%(asctime)s] %(levelname)s: %(message)s", "%H:%M:%S")

    handlers = []
    if log_path:
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
        except OSError:
            pass
        try:
            handlers.append(logging.FileHandler(log_path, mode="a", encoding="utf-8"))
        except OSError:
            pass
    handlers.append(logging.StreamHandler(sys.stdout))

    for handler in handlers:
        handler.setFormatter(fmt)
        logger.addHandler(handler)
    return logger


def _show_error(title, message, QtWidgets=None):
    if QtWidgets is not None:
        try:
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
            QtWidgets.QMessageBox.critical(None, title, message)
            return
        except Exception:
            pass
    try:
        from PyQt5 import QtWidgets as _QtWidgets  # type: ignore
        app = _QtWidgets.QApplication.instance() or _QtWidgets.QApplication(sys.argv)
        _QtWidgets.QMessageBox.critical(None, title, message)
        return
    except Exception:
        pass
    if os.name == "nt":
        try:
            ctypes.windll.user32.MessageBoxW(0, message, title, 0x00000010)
            return
        except Exception:
            pass
    print(f"{title}: {message}", file=sys.stderr)


def _install_exception_hook(logger, QtWidgets):
    def _handler(exc_type, exc_value, exc_tb):
        if exc_type is KeyboardInterrupt:
            sys.__excepthook__(exc_type, exc_value, exc_tb)
            return
        logger.error("Unhandled exception", exc_info=(exc_type, exc_value, exc_tb))
        _show_error("Teensy Live Plot", f"Unhandled error:\n{exc_value}", QtWidgets=QtWidgets)
    sys.excepthook = _handler


def reader(ser, out_q, stop_evt):
    try:
        while not stop_evt.is_set():
            line = ser.readline()
            if not line:
                continue
            out_q.put(line.decode("utf-8", "replace").strip())
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="COM7 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument(
        "--cols",
        nargs="*",
        default=["heading", "tiltX", "tiltY", "loopTime"],
        help="left-column plot names; must match CSV header",
    )
    ap.add_argument(
        "--cols-right",
        nargs="*",
        default=["throttleA", "rvAcc", "tvcX", "tvcY"],
        help="right-column plot names; must match CSV header",
    )
    ap.add_argument("--maxpts", type=int, default=2000)
    ap.add_argument(
        "--scroll-window",
        type=float,
        default=25.0,
        help="seconds shown on the x-axis; <=0 keeps expanding",
    )
    ap.add_argument("--log", help="write diagnostic output to this file")
    args = ap.parse_args()

    logger = _setup_logger(args.log)
    logger.info("Starting viewer on %s @ %d baud", args.port, args.baud)
    if args.log:
        logger.info("Logging to %s", args.log)

    try:
        from PyQt5 import QtWidgets, QtCore  # type: ignore
    except ModuleNotFoundError as exc:
        logger.error("PyQt5 not available: %s", exc)
        _show_error("Teensy Live Plot", f"PyQt5 is required but not installed:\n{exc}")
        return 1
    except Exception as exc:
        logger.error("Failed to import PyQt5: %s", exc)
        _show_error("Teensy Live Plot", f"Failed to import PyQt5:\n{exc}")
        return 1

    try:
        import pyqtgraph as pg  # type: ignore
    except ModuleNotFoundError as exc:
        logger.error("pyqtgraph not available: %s", exc)
        _show_error("Teensy Live Plot", f"pyqtgraph is required but not installed:\n{exc}", QtWidgets=QtWidgets)
        return 1
    except Exception as exc:
        logger.error("Failed to import pyqtgraph: %s", exc)
        _show_error("Teensy Live Plot", f"Failed to import pyqtgraph:\n{exc}", QtWidgets=QtWidgets)
        return 1

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    _install_exception_hook(logger, QtWidgets)

    try:
        import serial  # type: ignore
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except ModuleNotFoundError as exc:
        logger.error("pyserial not available: %s", exc)
        _show_error("Teensy Live Plot", f"pyserial is required but not installed:\n{exc}", QtWidgets=QtWidgets)
        return 1
    except Exception as exc:
        logger.error("Could not open serial port %s: %s", args.port, exc)
        try:
            from serial.tools import list_ports  # type: ignore
            ports = [port.device for port in list_ports.comports()]
        except Exception:
            ports = []
        if ports:
            hint = "Available ports: " + ", ".join(ports)
            logger.error(hint)
        else:
            hint = "No serial ports detected."
            logger.error(hint)
        _show_error(
            "Teensy Live Plot",
            f"Could not open serial port {args.port}:\n{exc}\n\n{hint}",
            QtWidgets=QtWidgets,
        )
        return 1

    # Start background reader
    q = queue.Queue()
    stop_evt = threading.Event()
    thr = threading.Thread(target=reader, args=(ser, q, stop_evt), daemon=True)
    thr.start()

    # Parse header (wait until we see a line starting with '#')
    header = None
    start_t = time.time()
    while time.time() - start_t < 5:
        try:
            s = q.get(timeout=0.1)
        except queue.Empty:
            continue
        if s.startswith("#"):
            header = [h.strip() for h in s.lstrip("#").split(",")]
            logger.info("Received header: %s", header)
            break
    if header is None:
        logger.warning("Did not receive a header line starting with '#'. Continuing anyway...")
        header = []

    # Figure out indices for requested columns
    requested_cols = []
    for name in list(args.cols) + list(args.cols_right):
        if name not in requested_cols:
            requested_cols.append(name)

    idx = {name: (header.index(name) if name in header else None) for name in requested_cols}
    missing_cols = [name for name, pos in idx.items() if pos is None]
    if missing_cols:
        logger.warning("Requested columns missing from header: %s", ", ".join(missing_cols))
    t_idx = header.index("t_us") if "t_us" in header else None

    win = QtWidgets.QWidget()
    win.setWindowTitle("Teensy Live Plot")
    layout = QtWidgets.QGridLayout(win)

    pg.setConfigOptions(antialias=True)

    y_ranges = {
        "heading": (-200, 200),
        "tiltX": (-30, 30),
        "tiltY": (-30, 30),
        "loopTime": (0.0, 0.1),
        "rvAcc": (-0.5, 3.5),
        "tvcX": (0, 3000),
        "tvcY": (0, 3000),
    }

    scroll_window = args.scroll_window

    plot_widgets = []
    signal_keys = []
    groups = [list(args.cols), list(args.cols_right)]

    for col_idx, group in enumerate(groups):
        for row_idx, name in enumerate(group):
            title = f"{name} vs time" if name else f"Signal {col_idx + 1}-{row_idx + 1}"
            widget = pg.PlotWidget(title=title)
            widget.showGrid(x=True, y=True, alpha=0.3)
            widget.enableAutoRange(x=scroll_window <= 0, y=False)
            target_range = y_ranges.get(name)
            if target_range:
                low, high = target_range
                widget.setYRange(low, high, padding=0)
                widget.setLimits(yMin=low, yMax=high)
            if scroll_window > 0:
                widget.setXRange(0, scroll_window, padding=0)
            plot_widgets.append(widget)
            signal_keys.append(name)
            layout.addWidget(widget, row_idx, col_idx, 1, 1)

    # Data buffers
    maxpts = args.maxpts
    t_buf = deque(maxlen=maxpts)
    data_buffers = [deque(maxlen=maxpts) for _ in signal_keys]

    # Curves
    curves = [widget.plot([]) for widget in plot_widgets]

    # Update timer
    def update():
        try:
            drained = 0
            while drained < 2000:
                try:
                    s = q.get_nowait()
                except queue.Empty:
                    break
                drained += 1
                if s.startswith("#") or not s:
                    continue
                row = [v.strip() for v in s.split(",")]
                try:
                    t = (float(row[t_idx]) * 1e-6) if t_idx is not None else time.monotonic()
                except Exception as exc:
                    logger.debug("Skipping row %s: %s", row, exc)
                    continue

                def sample_value(key):
                    if not key:
                        return float("nan")
                    col_idx = idx.get(key)
                    if col_idx is None or col_idx >= len(row):
                        return float("nan")
                    try:
                        return float(row[col_idx])
                    except Exception:
                        return float("nan")

                t_buf.append(t)
                for buf, key in zip(data_buffers, signal_keys):
                    buf.append(sample_value(key))

            if len(t_buf) > 2:
                t0 = t_buf[0]
                tt = [ti - t0 for ti in t_buf]
                for curve, buf in zip(curves, data_buffers):
                    values = list(buf)
                    if any(not math.isnan(v) for v in values):
                        curve.setData(tt, values)
                    else:
                        curve.clear()

                if scroll_window > 0 and tt:
                    xmin = max(0.0, tt[-1] - scroll_window)
                    xmax = xmin + scroll_window
                    for widget in plot_widgets:
                        widget.setXRange(xmin, xmax, padding=0)
        except Exception:
            logger.exception("Unhandled error in update loop")
            _show_error("Teensy Live Plot", "An internal error occurred. See log for details.", QtWidgets=QtWidgets)
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(16)  # ~60 FPS

    win.resize(1100, 800)
    win.show()
    try:
        rc = app.exec_()
        logger.info("Event loop exited with code %s", rc)
        return rc
    finally:
        stop_evt.set()
        logger.info("Stopping reader thread")
        thr.join(timeout=1)


if __name__ == "__main__":
    sys.exit(main())
