"""Read-only port of the Phase 4 integration (robotics_mapping::integrate + raycast::Supercover +
occupancy add_at/classify) and of the evaluator's sampling, for root-cause analysis. Validated
against each run's own final-grid.txt and map-metrics.yaml before any result is used."""
import csv, math, os
import numpy as np

W = H = 140
RES = 0.05
OX = OY = -3.5
FREE_MARK, OCC_MARK = 1, 2


def read_scans(run_dir):
    scans = []
    for line in open(os.path.join(run_dir, "integrated-scans.csv")):
        if line.startswith("#") or line.startswith("stamp"):
            continue
        p = line.strip().split(",")
        n = int(p[8])
        scans.append(dict(
            stamp=float(p[0]), x=float(p[1]), y=float(p[2]), th=float(p[3]),
            a0=float(p[4]), da=float(p[5]), rmin=float(p[6]), rmax=float(p[7]),
            ranges=np.array(p[9:9 + n], dtype=np.float32)))
    return scans


def read_params(run_dir):
    hdr = {}
    for line in open(os.path.join(run_dir, "final-grid.txt")):
        if line.strip() == "data":
            break
        k, _, v = line.partition(" ")
        hdr[k] = v.strip()
    return hdr


def read_grid(run_dir):
    L = open(os.path.join(run_dir, "final-grid.txt")).read().split("\n")
    i = L.index("data")
    return np.array([[np.float32(v) for v in l.split()] for l in L[i + 1:] if l.strip()], dtype=np.float32)


def supercover(x0, y0, x1, y1):
    u0, v0 = (x0 - OX) / RES, (y0 - OY) / RES
    u1, v1 = (x1 - OX) / RES, (y1 - OY) / RES
    ix, iy = math.floor(u0), math.floor(v0)
    ex, ey = math.floor(u1), math.floor(v1)
    dx, dy = u1 - u0, v1 - v0

    def axis(i, u, d):
        if d > 0.0:
            return 1, ((i + 1) - u) / d, 1.0 / d
        if d < 0.0:
            return -1, (i - u) / d, -1.0 / d
        return 0, math.inf, math.inf
    sx, tmx, tdx = axis(ix, u0, dx)
    sy, tmy, tdy = axis(iy, v0, dy)
    while True:
        yield ix, iy
        if ix == ex and iy == ey:
            return
        xo = ix != ex and sx != 0
        yo = iy != ey and sy != 0
        if xo and yo:
            if tmx < tmy:
                gx, gy = True, False
            elif tmy < tmx:
                gx, gy = False, True
            else:
                gx, gy = True, True
        else:
            gx, gy = xo, yo
        if not gx and not gy:
            return
        if gx:
            ix += sx
            tmx += tdx
        if gy:
            iy += sy
            tmy += tdy


def replay(scans, par, clear_margin=0.10):
    """Returns dict with final log-odds (float32 HxW, index [j][i]), per-cell counts of scans that
    marked the cell OCCUPIED / FREE, and one record per hit beam."""
    l_occ, l_free = np.float32(par["l_occ"]), np.float32(par["l_free"])
    l_min, l_max = np.float32(par["l_min"]), np.float32(par["l_max"])
    L = np.zeros(W * H, dtype=np.float32)
    n_occ = np.zeros(W * H, dtype=np.int32)
    n_free = np.zeros(W * H, dtype=np.int32)
    hits = []   # (scan, beam, range, ex, ey, cell index or -1)
    for si, s in enumerate(scans):
        marks = {}
        for bi, raw in enumerate(s["ranges"]):
            z = float(raw)
            th = s["th"] + s["a0"] + bi * s["da"]
            c, sn = math.cos(th), math.sin(th)
            if math.isnan(z) or z <= 0.0 or (math.isfinite(z) and z < s["rmin"]):
                continue
            if math.isfinite(z) and z < s["rmax"]:
                ex, ey = s["x"] + c * z, s["y"] + sn * z
                last_hit = True
            else:
                reach = s["rmax"] - clear_margin
                if not reach > s["rmin"]:
                    continue
                ex, ey = s["x"] + c * reach, s["y"] + sn * reach
                last_hit = False
            cells = list(supercover(s["x"], s["y"], ex, ey))
            end_inside = False
            for k, (i, j) in enumerate(cells):
                if not (0 <= i < W and 0 <= j < H):
                    break
                idx = j * W + i
                if k == len(cells) - 1:
                    end_inside = True
                    if last_hit:
                        marks[idx] = OCC_MARK
                else:
                    if marks.get(idx, 0) < FREE_MARK:
                        marks[idx] = FREE_MARK
            if last_hit:
                hits.append((si, bi, z, ex, ey, (cells[-1][1] * W + cells[-1][0]) if end_inside else -1))
        for idx, m in marks.items():
            if m == OCC_MARK:
                L[idx] = np.clip(np.float32(L[idx] + l_occ), l_min, l_max)
                n_occ[idx] += 1
            else:
                L[idx] = np.clip(np.float32(L[idx] + l_free), l_min, l_max)
                n_free[idx] += 1
    return dict(L=L.reshape(H, W), n_occ=n_occ.reshape(H, W), n_free=n_free.reshape(H, W), hits=hits)


PILLARS = [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)]
R_P = 0.15


def true_dist(x, y):
    """Distance to the nearest true surface (walls are segments at +/-3.0; pillars are circles)."""
    def seg(ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
        return math.hypot(x - (ax + t * dx), y - (ay + t * dy))
    d = min(seg(-3, -3, 3, -3), seg(3, -3, 3, 3), seg(3, 3, -3, 3), seg(-3, 3, -3, -3))
    for px, py in PILLARS:
        d = min(d, abs(math.hypot(x - px, y - py) - R_P))
    return d


def cell_centre(i, j):
    return OX + (i + 0.5) * RES, OY + (j + 0.5) * RES


# ---------------------------------------------------------------- evaluator port
def make_samples(spacing=0.01):
    """(x, y, nx, ny, owner) with owner ('w',k) or ('p',k); same construction as GroundTruth::surface_samples."""
    walls = [((-3, -3), (3, -3), (0, 1)), ((3, -3), (3, 3), (-1, 0)), ((3, 3), (-3, 3), (0, -1)), ((-3, 3), (-3, -3), (1, 0))]
    out = []
    for k, (a, b, n) in enumerate(walls):
        ln = math.hypot(b[0] - a[0], b[1] - a[1])
        cnt = max(1, math.ceil(ln / spacing))
        for s in range(cnt):
            t = (s + 0.5) / cnt
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, n[0], n[1], ("w", k)))
    for k, (cx, cy) in enumerate(PILLARS):
        cnt = max(8, math.ceil(math.tau * R_P / spacing))
        for s in range(cnt):
            a = math.tau * (s + 0.5) / cnt
            nx, ny = math.cos(a), math.sin(a)
            out.append((cx + nx * R_P, cy + ny * R_P, nx, ny, ("p", k)))
    return out


def visibility(samples, pos, max_range=3.5):
    """bool matrix [n_samples, n_positions]: the evaluator's is_visible, vectorised.
    Blocking: other pillars (a segment closer than r - 1e-9 to a pillar centre); the room's other walls
    cannot block a segment between two points inside the room."""
    S = np.array([[s[0], s[1], s[2], s[3]] for s in samples])
    sx, sy, nx, ny = S[:, 0:1], S[:, 1:2], S[:, 2:3], S[:, 3:4]
    ox, oy = pos[:, 0][None, :], pos[:, 1][None, :]
    tx, ty = ox - sx, oy - sy
    dist = np.hypot(tx, ty)
    vis = (dist <= max_range) & ((nx * tx + ny * ty) > 1e-9)
    for k, (cx, cy) in enumerate(PILLARS):
        own = np.array([s[4] == ("p", k) for s in samples])[:, None]
        # distance from the pillar centre to the segment sample -> position
        dx, dy = ox - sx, oy - sy
        L2 = dx * dx + dy * dy
        with np.errstate(invalid="ignore", divide="ignore"):
            t = np.clip(((cx - sx) * dx + (cy - sy) * dy) / np.where(L2 == 0, 1, L2), 0, 1)
        d = np.hypot(cx - (sx + t * dx), cy - (sy + t * dy))
        vis &= own | ~(d < R_P - 1e-9)
    return vis


def edt_to_occupied(occ):
    """exact distance (m) from every cell centre to the nearest occupied cell centre."""
    js, is_ = np.nonzero(occ)
    gj, gi = np.mgrid[0:H, 0:W]
    d = np.full((H, W), np.inf)
    for j, i in zip(js, is_):
        d = np.minimum(d, np.hypot((gi - i) * RES, (gj - j) * RES))
    return d


def nearest_cell(x, y):
    fx, fy = (x - OX) / RES - 0.5, (y - OY) / RES - 0.5
    i = min(max(math.floor(fx + 0.5), 0), W - 1)
    j = min(max(math.floor(fy + 0.5), 0), H - 1)
    return i, j


def closed_cells(x, y):
    def axis(v, o, n):
        f = (v - o) / RES
        r = math.floor(f + 0.5)
        idx = [math.floor(f)]
        if abs(f - r) < 1e-4:
            idx = [r - 1, r]
        return [k for k in idx if 0 <= k < n]
    return [(i, j) for i in axis(x, OX, W) for j in axis(y, OY, H)]


def read_traj(run_dir):
    rows = list(csv.DictReader(open(os.path.join(run_dir, "trajectory.csv"))))
    return np.array([[float(r["x"]), float(r["y"])] for r in rows])


def grade(L, par, traj, tau=0.05, spacing=0.01):
    """Reproduce the evaluator: returns dict with the per-sample arrays and the metrics."""
    occ_thr, free_thr = float(par["occupied_threshold"]), float(par["free_threshold"])
    occ = L >= np.float32(par["occupied_threshold"])
    free = L <= np.float32(par["free_threshold"])
    samples = make_samples(spacing)
    vis = visibility(samples, traj)
    observable = vis.any(axis=1)
    D = edt_to_occupied(occ)
    found = np.zeros(len(samples), bool)
    false_free = np.zeros(len(samples), bool)
    for n, s in enumerate(samples):
        if not observable[n]:
            continue
        i, j = nearest_cell(s[0], s[1])
        found[n] = occ.any() and D[j, i] <= tau + 1e-9
        cells = closed_cells(s[0], s[1])
        false_free[n] = bool(cells) and all(free[j2, i2] for i2, j2 in cells)
    return dict(samples=samples, vis=vis, observable=observable, found=found, false_free=false_free, D=D, occ=occ, free=free,
                completeness=found[observable].sum() / observable.sum(), n_obs=int(observable.sum()),
                n_false_free=int(false_free.sum()))
