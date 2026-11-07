"""XT-M60 target viewer: XTM5, XYZ in sensor coordinates (metres)."""
import argparse
import socket
import struct
import threading
import time

HEADER = struct.Struct('<4sIQHHIIII')
TARGET = struct.Struct('<IfffIIIffffI')
MAX_POINTS = 1_000_000
AUTO_STATUS = struct.Struct('<4sIIIIIIf')


def read_exact(sock, length, stop):
    result = bytearray()
    deadline = time.monotonic() + 5
    while len(result) < length and not stop.is_set():
        if time.monotonic() > deadline:
            raise TimeoutError('Frame receive timeout')
        try:
            chunk = sock.recv(min(length - len(result), 65536))
        except socket.timeout:
            continue
        if not chunk:
            raise EOFError('Sender disconnected')
        result.extend(chunk)
    if stop.is_set():
        raise EOFError('Viewer closing')
    return bytes(result)


def check_header(data):
    magic, length, frame_id, width, height, count, ntargets, bg, ready = HEADER.unpack(data)
    if (magic != b'XTM5' or not width or not height or
            width * height > MAX_POINTS or count > width * height or
            ntargets > count + 16 or length != count * 20 + ntargets * TARGET.size or
            bg != 0 or ready != 1):
        raise ValueError('Invalid XTM5 frame; check sender version')
    return length, frame_id, width, height, count, ntargets, bg, ready


def decode_payload(payload, count, ntargets):
    import math
    import numpy as np
    dtype = np.dtype([('values', '<f4', (4,)), ('id', '<u4')])
    records = np.frombuffer(payload, dtype=dtype, count=count)
    points = records['values'].copy()
    labels = records['id'].copy()
    valid = np.isfinite(points).all(axis=1)
    valid &= (points[:, 3] >= 1500) & (points[:, 3] < 964000)
    targets = [TARGET.unpack_from(payload, count * 20 + i * TARGET.size)
               for i in range(ntargets)]
    ids = set()
    for tid, x, y, z, size, age, is_new, span_x, span_y, span_z, nearest, geometry_valid in targets:
        if (not tid or tid in ids or not all(map(math.isfinite, (x, y, z))) or
                size > count or not age or is_new not in (0, 1) or
                geometry_valid not in (1, 3, 7, 15) or (size == 0) != (geometry_valid == 15) or min(span_x, span_y, span_z, nearest) < 0 or
                not all(map(math.isfinite, (span_x, span_y, span_z, nearest)))):
            raise ValueError('Invalid target record')
        ids.add(tid)
    if any(int(tid) not in ids for tid in np.unique(labels) if tid):
        raise ValueError('Unknown point target ID')
    return points[valid], labels[valid], targets


def target_state(t):
    return {1: 'Geometry', 3: 'Candidate', 7: 'Tracking', 15: 'Predicted'}[t[11]]


def target_row(t):
    row = [('T' + str(t[0] & 0x7fffffff)) if t[11] & 2 else ('G' + str(t[0])), target_state(t), *(f'{value:.3f}' for value in t[1:4]),
           *(f'{value:.3f}' for value in t[7:11]), str(t[4]), str(t[5])]
    if t[11] == 15:
        row[8] = '-'  # No measured nearest distance for a prediction.
    return row


def rectangle_indices(points, matrix, width, height, bounds):
    """Select projected point centres, including overlapping front/back points.

    matrix is the row-major display model-view-projection matrix.
    Bounds and widget size are in logical pixels (also on HiDPI displays).
    """
    import numpy as np
    xyz = points[:, [0, 2, 1]]
    homogeneous = np.column_stack((xyz, np.ones(len(xyz))))
    clip = homogeneous @ np.asarray(matrix, dtype=float).T
    good = np.isfinite(clip).all(axis=1) & (clip[:, 3] > 0)
    ndc = np.full((len(points), 3), np.nan)
    np.divide(clip[:, :3], clip[:, 3, None], out=ndc, where=good[:, None])
    good &= (np.abs(ndc) <= 1).all(axis=1)
    sx = (ndc[:, 0] + 1) * width / 2
    sy = (1 - ndc[:, 1]) * height / 2
    left, top, right, bottom = bounds
    good &= (sx >= left) & (sx <= right) & (sy >= top) & (sy <= bottom)
    return np.flatnonzero(good)


def read_frame(client, stop):
    import math
    magic = read_exact(client, 4, stop)
    if magic != b'XTO1':
        raise ValueError('Sender version mismatch: build and restart moving-obstacles sender and viewer together')
    raw = magic + read_exact(client, AUTO_STATUS.size-4, stop)
    _, total, accepted, rejected, edges, labelled, valid, elapsed = AUTO_STATUS.unpack(raw)
    if (total > valid or accepted + rejected != total or labelled > valid or
            valid > MAX_POINTS or edges > 8 * valid or not math.isfinite(elapsed) or elapsed < 0):
        raise ValueError('Invalid cluster status')
    automatic = (total, accepted, rejected, edges, labelled, valid, elapsed)
    info = check_header(read_exact(client, HEADER.size, stop))
    length, frame_id, width, height, count, ntargets, bg, ready = info
    if valid > count:
        raise ValueError('Cluster status does not match frame')
    points, labels, targets = decode_payload(read_exact(client, length, stop), count, ntargets)
    if accepted != sum(t[11] != 15 for t in targets):
        raise ValueError("Observed component count mismatch")
    if int((labels != 0).sum()) != labelled:
        raise ValueError('Cluster label count mismatch')
    for target in targets:
        if int((labels == target[0]).sum()) != target[4]:
            raise ValueError('Component point count mismatch')
    return frame_id, width, height, points, labels, targets, bg, ready, automatic


def full_frame_text(frame):
    """Export all transmitted points, independent of camera projection/selection."""
    fid, width, height, points, labels, targets, bg, ready, automatic = frame
    lines = [f'# Frame {fid} | FULL transmitted frame | points {len(points)}',
             f'# Sensor dimensions {width} x {height}',
             '# Sensor XYZ metres; SDK depth millimetres; no camera transform applied.',
             '# Filtered point index is NOT original sensor pixel index. Invalid/near points were not transmitted.',
             f'# Cluster status {automatic!r}',
             f'# Target records (id, XYZ, points, hits, new, spans, nearest, flags): {targets!r}',
             'Frame point index\tX (m)\tY (m)\tZ (m)\tSDK depth (mm)\tFrame component']
    for i, (point, label) in enumerate(zip(points, labels)):
        lines.append('\t'.join([str(i), *(format(float(v), '.9g') for v in point), str(int(label))]))
    return '\n'.join(lines) + '\n'


def main():
    import numpy as np
    from PyQt5 import QtWidgets, QtCore, QtGui
    import pyqtgraph.opengl as gl

    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='10.0.0.2')
    parser.add_argument('--port', type=int, default=9000)
    args = parser.parse_args()
    app = QtWidgets.QApplication([])
    stop = threading.Event()
    reset_request = threading.Event()
    lock = threading.Lock()
    state = {'latest': None, 'message': 'Starting listener'}

    def message(text):
        with lock:
            state['message'] = text
        print(text, flush=True)

    def receive():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((args.bind, args.port))
                server.listen(1)
                server.settimeout(1)
                message(f'Listening on {args.bind}:{args.port}; waiting for Zybo')
                while not stop.is_set():
                    try:
                        client, address = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(1)
                        message(f'Connected: {address[0]}')
                        last_target_log = 0.
                        try:
                            while not stop.is_set():
                                if reset_request.is_set():
                                    client.sendall(b'B')
                                    reset_request.clear()
                                frame_id, width, height, points, labels, targets, bg, ready, automatic = read_frame(client, stop)
                                now = time.monotonic()
                                log_positions = now - last_target_log >= 1.
                                if log_positions:
                                    for t in targets:
                                        print(f'BOXTRACK frame={frame_id} id={t[0]} state={target_state(t)} '
                                              f'nearest={t[10]:.3f}m observed XYZ spans='
                                              f'{t[7]:.3f},{t[8]:.3f},{t[9]:.3f}m points={t[4]}', flush=True)
                                if log_positions:
                                    last_target_log = now
                                with lock:
                                    # One slot: the next frame replaces an undisplayed frame.
                                    state['latest'] = (frame_id, width, height, points, labels, targets, bg, ready, automatic)
                                    state['message'] = 'Receiving'
                        except (OSError, EOFError, ValueError, TimeoutError) as error:
                            if not stop.is_set():
                                message(f'{error}; waiting for reconnect')
        except OSError as error:
            message(f'Listener error: {error}')

    window = QtWidgets.QWidget()
    window.setWindowTitle('XT-M60 | BOX TRACK v1')
    window.resize(1150, 800)
    layout = QtWidgets.QVBoxLayout(window)
    status = QtWidgets.QLabel('Waiting for Zybo')
    layout.addWidget(status)
    class RegionView(gl.GLViewWidget):
        def __init__(self):
            super().__init__()
            self.origin = None
            self.on_start = None
            self.on_selected = None
            self.band = QtWidgets.QRubberBand(QtWidgets.QRubberBand.Rectangle, self)

        def mousePressEvent(self, event):
            if event.button() == QtCore.Qt.LeftButton and event.modifiers() & QtCore.Qt.ShiftModifier:
                if self.on_start is not None and self.on_start():
                    self.origin = event.pos()
                    self.band.setGeometry(QtCore.QRect(self.origin, QtCore.QSize()))
                    self.band.show()
                event.accept()
                return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event):
            if self.origin is not None:
                self.band.setGeometry(QtCore.QRect(self.origin, event.pos()).normalized())
                event.accept()
                return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event):
            if self.origin is not None and event.button() == QtCore.Qt.LeftButton:
                rect = QtCore.QRect(self.origin, event.pos()).normalized().intersected(self.rect())
                self.origin = None
                self.band.hide()
                event.accept()
                if rect.width() >= 3 and rect.height() >= 3 and self.on_selected is not None:
                    self.on_selected(rect)
                return
            super().mouseReleaseEvent(event)

        def wheelEvent(self, event):
            if self.origin is not None:
                event.accept()
                return
            super().wheelEvent(event)

    view = RegionView()
    view.setCameraPosition(distance=4, elevation=15, azimuth=-90)
    layout.addWidget(view, 1)
    cloud = gl.GLScatterPlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                size=3, color=(0.2, 0.8, 1., 1.), pxMode=True)
    cloud.setGLOptions('opaque')
    view.addItem(cloud)
    centers = gl.GLScatterPlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                  size=12, color=(1., 0., 1., 1.), pxMode=True)
    centers.setGLOptions('opaque')
    view.addItem(centers)
    selected_cloud = gl.GLScatterPlotItem(pos=np.empty((0, 3), dtype=np.float32),
                                         size=7, color=(0., 1., 1., 1.), pxMode=True)
    selected_cloud.setGLOptions('opaque')
    view.addItem(selected_cloud)
    pause_button = QtWidgets.QPushButton('Pause display / select region with Shift + drag')
    layout.addWidget(pause_button)
    fit_button = QtWidgets.QPushButton('Fit current frame')
    layout.addWidget(fit_button)
    full_frame_button = QtWidgets.QPushButton('Save FULL frame (.txt) - no selection needed')
    layout.addWidget(full_frame_button)
    reset_button = QtWidgets.QPushButton('Reset box tracks')
    reset_button.clicked.connect(lambda: reset_request.set())
    layout.addWidget(reset_button)
    auto_note = QtWidgets.QLabel('Automatic scan starts on connection. Shift + drag only inspects data; no ROI is required.')
    auto_note.setWordWrap(True)
    layout.addWidget(auto_note)
    table = QtWidgets.QTableWidget(0, 11)
    table.setHorizontalHeaderLabels(['ID', 'State', 'X (m)', 'Y (m)', 'Z (m)',
                                     'X span (m)', 'Y span (m)', 'Z span (m)',
                                     'Nearest XYZ (m)', 'Points', 'Hits'])
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
    table.setMaximumHeight(150)
    layout.addWidget(table)
    note = QtWidgets.QLabel('Shift + drag: region data (auto pause) | Cyan: selected | Drag: rotate | Wheel: zoom | Gray: geometry | Colour: persistent box ID | Orange marker: prediction only | XYZ: sensor-relative | G IDs reset each frame | Minimum depth: 1.5 m')
    note.setWordWrap(True)
    layout.addWidget(note)
    current = None
    paused = False
    fitted = False
    display_count = 0
    rate_time = time.monotonic()
    display_rate = 0.
    last_display_frame = time.monotonic()

    def fit():
        nonlocal fitted
        if current is None or not len(current[3]):
            return
        xyz = current[3][:, [0, 2, 1]]  # Sensor Y up -> display Z up.
        low, high = np.percentile(xyz, [2, 98], axis=0)
        center = (low + high) / 2
        span = max(float(np.max(high - low)), .2)
        view.setCameraPosition(pos=QtGui.QVector3D(*map(float, center)),
                               distance=span * 2.2, elevation=15, azimuth=-90)
        fitted = True

    fit_button.clicked.connect(fit)

    def set_paused(value):
        nonlocal paused
        paused = value
        pause_button.setText('Resume live display' if paused else
                             'Pause display / select region with Shift + drag')
        if not paused:
            selected_cloud.setData(pos=np.empty((0, 3), dtype=np.float32))

    pause_button.clicked.connect(lambda: set_paused(not paused))

    def save_full_frame():
        if current is None:
            QtWidgets.QMessageBox.information(window, 'Full frame', 'No frame received yet.')
            return
        frame = current
        set_paused(True)
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            window, 'Save FULL transmitted frame', f'full_frame_{frame[0]}.txt', 'Text files (*.txt)')
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8', newline='\n') as output:
                    output.write(full_frame_text(frame))
                QtWidgets.QMessageBox.information(window, 'Saved',
                    f'Saved all {len(frame[3])} transmitted points. Resume live display before the next capture.')
            except Exception as error:
                QtWidgets.QMessageBox.warning(window, 'Save error', str(error))

    full_frame_button.clicked.connect(save_full_frame)

    def begin_selection():
        if current is None:
            return False
        set_paused(True)
        return True

    def inspect_region(rect):
        if current is None:
            return
        try:
            viewport = view.getViewport()
            # pyqtgraph 0.14+ takes (region, viewport); older versions take region.
            try:
                projection = view.projectionMatrix(viewport, viewport)
            except TypeError:
                projection = view.projectionMatrix(viewport)
            transform = projection * view.viewMatrix()
            matrix_rows = [transform.row(r) for r in range(4)]
            matrix = np.asarray([[v.x(), v.y(), v.z(), v.w()] for v in matrix_rows], dtype=float)
            bounds = (rect.left(), rect.top(), rect.right(), rect.bottom())
            indices = rectangle_indices(current[3], matrix, view.width(), view.height(), bounds)
        except Exception as error:
            QtWidgets.QMessageBox.warning(window, 'Selection error', str(error))
            return
        frame = current
        points = frame[3][indices].copy()
        labels = frame[4][indices].copy()
        selected_cloud.setData(pos=points[:, [0, 2, 1]])
        if not len(points):
            QtWidgets.QMessageBox.information(window, 'Region data',
                                              'No transmitted valid points in this rectangle. Display remains paused.')
            return

        headings = ['Frame point index', 'X (m)', 'Y (m)', 'Z (m)', 'SDK depth (mm)', 'Frame component']
        rows = [[str(int(i)), *(format(float(v), '.9g') for v in p), str(int(label))]
                for i, p, label in zip(indices, points, labels)]
        summary = [f'Frame {frame[0]} | selected {len(points)} / {len(frame[3])} valid points',
                   'Sensor XYZ; SDK depth is separate from Z. All overlapping depths are included.',
                   'Point index is within the filtered frame, NOT an original sensor pixel index.',
                   'Metric                  min          median           max']
        for column, name in enumerate(['X (m)', 'Y (m)', 'Z (m)', 'SDK depth (mm)']):
            values = points[:, column]
            summary.append(f'{name:16s} {np.min(values):13.6f} {np.median(values):13.6f} {np.max(values):13.6f}')
        export_text = ('\n'.join('# ' + line for line in summary) +
                       '\n' + '\t'.join(headings) + '\n' +
                       '\n'.join('\t'.join(row) for row in rows) + '\n')

        class PointModel(QtCore.QAbstractTableModel):
            def rowCount(self, parent=QtCore.QModelIndex()):
                return 0 if parent.isValid() else len(rows)

            def columnCount(self, parent=QtCore.QModelIndex()):
                return 0 if parent.isValid() else len(headings)

            def data(self, index, role=QtCore.Qt.DisplayRole):
                if index.isValid() and role == QtCore.Qt.DisplayRole:
                    return rows[index.row()][index.column()]
                return None

            def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
                if role == QtCore.Qt.DisplayRole:
                    return headings[section] if orientation == QtCore.Qt.Horizontal else str(section + 1)
                return None

        dialog = QtWidgets.QDialog(window)
        dialog.setWindowTitle(f'Region data - frame {frame[0]} - {len(points)} points')
        dialog.resize(960, 640)
        box = QtWidgets.QVBoxLayout(dialog)
        summary_label = QtWidgets.QLabel('\n'.join(summary))
        summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        summary_label.setFont(QtGui.QFont('monospace', 10))
        box.addWidget(summary_label)
        data_table = QtWidgets.QTableView()
        model = PointModel(data_table)
        data_table.setModel(model)
        data_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        box.addWidget(data_table, 1)
        buttons = QtWidgets.QHBoxLayout()
        box.addLayout(buttons)
        copy_button = QtWidgets.QPushButton('Copy ALL selected data')
        copy_button.clicked.connect(lambda: app.clipboard().setText(export_text))
        buttons.addWidget(copy_button)
        save_button = QtWidgets.QPushButton('Save selected data (.txt)')
        buttons.addWidget(save_button)

        def save_text():
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(
                dialog, 'Save region data', f'region_frame_{frame[0]}.txt', 'Text files (*.txt)')
            if filename:
                try:
                    with open(filename, 'w', encoding='utf-8', newline='\n') as output:
                        output.write(export_text)
                except OSError as error:
                    QtWidgets.QMessageBox.warning(dialog, 'Save failed', str(error))

        save_button.clicked.connect(save_text)
        close_button = QtWidgets.QPushButton('Close (keep paused)')
        close_button.clicked.connect(dialog.accept)
        buttons.addWidget(close_button)
        dialog.exec_()

    view.on_start = begin_selection
    view.on_selected = inspect_region

    def update():
        nonlocal current, display_count, rate_time, display_rate, last_display_frame
        with lock:
            frame = None if paused else state['latest']
            if not paused:
                state['latest'] = None
            msg = state['message']
        if frame is not None:
            last_display_frame = time.monotonic()
            current = frame
            points = frame[3]
            colours = np.ones((len(points), 4), dtype=np.float32)
            colours[:] = (.35, .48, .55, 1.)
            palette = np.asarray([(0.4, 1., .3, 1.), (.7, .5, 1., 1.),
                                  (1., .4, .7, 1.), (.3, .7, 1., 1.)], dtype=np.float32)
            targets = frame[5]
            for t in targets:
                if t[11] & 2:
                    colours[frame[4] == t[0]] = palette[t[0] % len(palette)]
            cloud.setData(pos=points[:, [0, 2, 1]], color=colours)
            box_targets = [t for t in targets if t[11] & 2]
            positions = np.asarray([[t[1], t[3], t[2]] for t in box_targets],
                                   dtype=np.float32).reshape(-1, 3)
            marker_colours = np.asarray([(1., .5, 0., 1.) if t[11] == 15 else
                                         ((1., 1., 0., 1.) if t[11] == 3 else (1., 0., 1., 1.))
                                         for t in box_targets], dtype=np.float32).reshape(-1, 4)
            centers.setData(pos=positions, color=marker_colours)
            table.setRowCount(len(targets))
            for row, t in enumerate(targets):
                values = target_row(t)
                for column, value in enumerate(values):
                    table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
            if not fitted:
                fit()
            display_count += 1
        elapsed = time.monotonic() - rate_time
        if elapsed >= 1:
            display_rate = display_count / elapsed
            display_count = 0
            rate_time = time.monotonic()
        if current is None:
            status.setText(msg)
        else:
            fid, width, height, points, labels, targets, bg, ready, automatic = current
            total, accepted, rejected, edges, labelled, valid, elapsed_ms = automatic
            watch = f'Boxes {sum(t[11] == 7 for t in targets)} | candidates {sum(t[11] == 3 for t in targets)} | predicted {sum(t[11] == 15 for t in targets)}'
            auto_note.setText(f'BOX TRACK v1 | labelled {labelled}/{valid} | '
                              f'detector {elapsed_ms:.1f} ms | observed sensor-axis sizes | '
                              '3 consecutive hits confirm; prediction expires after 0.8 s. Relative motion, not world speed.')
            mode = 'PAUSED (receiver continues)' if paused else msg
            if not paused and time.monotonic() - last_display_frame > 1.:
                mode = 'STALE FRAME - no current tracking data | ' + msg
                centers.setData(pos=np.empty((0, 3), dtype=np.float32))
            status.setText(f'{mode} | frame {fid} | {width}x{height} | '
                           f'valid {len(points)}/{width*height} | {watch} | display {display_rate:.1f} fps')

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    worker = threading.Thread(target=receive, daemon=True)
    worker.start()
    app.aboutToQuit.connect(stop.set)
    window.show()
    result = app.exec_()
    stop.set()
    worker.join(timeout=2)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
