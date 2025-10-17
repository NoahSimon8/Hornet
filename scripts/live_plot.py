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
    ap.add_argument("--cols", nargs="*", default=["throttleA","throttleB","tvcX","tvcY","rvAcc","pitch","roll","tiltX","tiltY","heading","x","y","z","xVel","yVel","zVel","desPitch","desRoll","desZVel","desTiltX","desTiltY","desHeading","time"],
                    help="names to plot; must match CSV header")
    ap.add_argument("--maxpts", type=int, default=2000)
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
    idx = {name: (header.index(name) if name in header else None) for name in args.cols}
    missing_cols = [name for name, pos in idx.items() if pos is None]
    if missing_cols:
        logger.warning("Requested columns missing from header: %s", ", ".join(missing_cols))
    t_idx = header.index("t_us") if "t_us" in header else None

    win = QtWidgets.QWidget()
    win.setWindowTitle("Teensy Live Plot")
    layout = QtWidgets.QGridLayout(win)

    pg.setConfigOptions(antialias=True)

    # Plots: XY path + Z over time + X over time + Y over time
    plt_xy = pg.PlotWidget(title="XY path")
    plt_x = pg.PlotWidget(title=f"{args.cols[0]} vs time")
    plt_y = pg.PlotWidget(title=f"{args.cols[1]} vs time" if len(args.cols) > 1 else "Y vs time")
    plt_z = pg.PlotWidget(title=f"{args.cols[2]} vs time" if len(args.cols) > 2 else "Z vs time")

    for p in (plt_xy, plt_x, plt_y, plt_z):
        p.showGrid(x=True, y=True, alpha=0.3)

    layout.addWidget(plt_xy, 0, 0, 2, 1)
    layout.addWidget(plt_z, 0, 1, 1, 1)
    layout.addWidget(plt_x, 1, 1, 1, 1)
    layout.addWidget(plt_y, 2, 0, 1, 2)

    # Data buffers
    maxpts = args.maxpts
    t_buf = deque(maxlen=maxpts)
    x_buf = deque(maxlen=maxpts)
    y_buf = deque(maxlen=maxpts)
    z_buf = deque(maxlen=maxpts)

    # Curves
    curve_xy = plt_xy.plot([], [], pen=None, symbol="o", symbolSize=3)
    curve_x = plt_x.plot([])
    curve_y = plt_y.plot([])
    curve_z = plt_z.plot([])

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
                    xv = float(row[idx.get(args.cols[0])]) if idx.get(args.cols[0]) is not None else float("nan")
                    yv = float(row[idx.get(args.cols[1])]) if len(args.cols) > 1 and idx.get(args.cols[1]) is not None else float("nan")
                    zv = float(row[idx.get(args.cols[2])]) if len(args.cols) > 2 and idx.get(args.cols[2]) is not None else float("nan")
                except Exception as exc:
                    logger.debug("Skipping row %s: %s", row, exc)
                    continue

                t_buf.append(t)
                x_buf.append(xv)
                y_buf.append(yv)
                z_buf.append(zv)

            if len(t_buf) > 2:
                t0 = t_buf[0]
                tt = [ti - t0 for ti in t_buf]
                xs = list(x_buf)
                ys = list(y_buf)
                zs = list(z_buf)

                has_x = any(not math.isnan(v) for v in xs)
                has_y = any(not math.isnan(v) for v in ys)
                has_z = any(not math.isnan(v) for v in zs)

                if has_x:
                    curve_x.setData(tt, xs)
                else:
                    curve_x.clear()
                if has_y:
                    curve_y.setData(tt, ys)
                else:
                    curve_y.clear()
                if has_z:
                    curve_z.setData(tt, zs)
                else:
                    curve_z.clear()

                xy_pairs = [(x, y) for x, y in zip(xs, ys) if not math.isnan(x) and not math.isnan(y)]
                if xy_pairs:
                    xs_finite, ys_finite = zip(*xy_pairs)
                    curve_xy.setData(xs_finite, ys_finite)
                else:
                    curve_xy.clear()
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
