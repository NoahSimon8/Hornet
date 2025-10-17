# tools/live_plot.py
import argparse, sys, threading, queue, time, csv
from collections import deque
import serial  # pip install pyserial

# UI
import pyqtgraph as pg            # pip install pyqtgraph
from PyQt5 import QtWidgets, QtCore  # pip install PyQt5

def reader(port, baud, out_q, stop_evt):
    ser = serial.Serial(port, baud, timeout=0.1)
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
    ap.add_argument("--cols", nargs="*", default=["pos_x","pos_y","pos_z"], 
                    help="names to plot; must match CSV header")
    ap.add_argument("--maxpts", type=int, default=2000)
    args = ap.parse_args()

    # Start background reader
    q = queue.Queue()
    stop_evt = threading.Event()
    thr = threading.Thread(target=reader, args=(args.port,args.baud,q,stop_evt), daemon=True)
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
            break
    if header is None:
        print("Did not receive a header line starting with '#'. Continuing anyway...")
        header = []

    # Figure out indices for requested columns
    idx = {name: (header.index(name) if name in header else None) for name in args.cols}
    t_idx = header.index("t_us") if "t_us" in header else None

    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QWidget()
    win.setWindowTitle("Teensy Live Plot")
    layout = QtWidgets.QGridLayout(win)

    pg.setConfigOptions(antialias=True)

    # Plots: XY path + Z over time + X over time + Y over time
    plt_xy = pg.PlotWidget(title="XY path")
    plt_x  = pg.PlotWidget(title=f"{args.cols[0]} vs time")
    plt_y  = pg.PlotWidget(title=f"{args.cols[1]} vs time" if len(args.cols)>1 else "Y vs time")
    plt_z  = pg.PlotWidget(title=f"{args.cols[2]} vs time" if len(args.cols)>2 else "Z vs time")

    for p in (plt_xy, plt_x, plt_y, plt_z):
        p.showGrid(x=True, y=True, alpha=0.3)

    layout.addWidget(plt_xy, 0, 0, 2, 1)
    layout.addWidget(plt_z,  0, 1, 1, 1)
    layout.addWidget(plt_x,  1, 1, 1, 1)
    layout.addWidget(plt_y,  2, 0, 1, 2)

    # Data buffers
    maxpts = args.maxpts
    t_buf = deque(maxlen=maxpts)
    x_buf = deque(maxlen=maxpts)
    y_buf = deque(maxlen=maxpts)
    z_buf = deque(maxlen=maxpts)

    # Curves
    curve_xy = plt_xy.plot([], [], pen=None, symbol='o', symbolSize=3)
    curve_x  = plt_x.plot([])
    curve_y  = plt_y.plot([])
    curve_z  = plt_z.plot([])

    # Update timer
    def update():
        # Drain queue quickly
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
                # time axis: use device timestamp if present, else host monotonic
                t = (float(row[t_idx]) * 1e-6) if t_idx is not None else time.monotonic()
                xv = float(row[idx.get(args.cols[0])] ) if idx.get(args.cols[0]) is not None else float('nan')
                yv = float(row[idx.get(args.cols[1])] ) if len(args.cols)>1 and idx.get(args.cols[1]) is not None else float('nan')
                zv = float(row[idx.get(args.cols[2])] ) if len(args.cols)>2 and idx.get(args.cols[2]) is not None else float('nan')
            except Exception:
                continue

            t_buf.append(t); x_buf.append(xv); y_buf.append(yv); z_buf.append(zv)

        if len(t_buf) > 2:
            # Normalize time axis to start at 0
            t0 = t_buf[0]
            tt = [ti - t0 for ti in t_buf]
            curve_x.setData(tt, list(x_buf))
            curve_y.setData(tt, list(y_buf))
            curve_z.setData(tt, list(z_buf))
            curve_xy.setData(list(x_buf), list(y_buf))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(16)  # ~60 FPS

    win.resize(1100, 800)
    win.show()
    rc = app.exec_()
    stop_evt.set()
    sys.exit(rc)

if __name__ == "__main__":
    main()
