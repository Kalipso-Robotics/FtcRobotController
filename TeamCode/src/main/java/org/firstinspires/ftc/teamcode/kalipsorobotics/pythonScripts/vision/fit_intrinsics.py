"""
Solve the Arducam intrinsics from tape-measured ground truth, no checkerboard.

WHY THIS EXISTS
    CameraIntrinsics.ARDUCAM was calibrated at 1280x800 (where fx ~= fy ~= 888,
    i.e. square pixels, which is correct) and then rescaled to 640x480 by
    multiplying fx,cx by 0.5 but fy,cy by 0.6. That is only valid if the camera
    anamorphically squashes the whole sensor into 640x480. If it crops instead,
    fy is about 19% wrong and every projected distance is biased. cy_orig also
    came out at 354 where ~400 was expected.

    Rather than argue about it, measure it. With the camera level at height h,
    the floor-contact pixel row of an object at forward distance dz obeys

        pixelY = cy + fy * (h / dz)          -- LINEAR in (1/dz)

    so regressing pixelY against 1/dz gives cy as the intercept and fy*h as the
    slope. Divide the slope by the ruler-measured h and fy falls out.

    The same CSV pins the other two axes:
        bboxW = D*fx / L    -> slope over 1/L gives fx   (D = ball diameter)
        bboxH = D*fy / L    -> slope over 1/L gives fy   (INDEPENDENT check)
        pixelX = cx - fx*(dx/dz)  -> intercept gives cx, slope gives -fx

    Three independent estimates of fy that agree is a real result. Three that
    disagree tells you which assumption broke.

USAGE
    python fit_intrinsics.py RaytracingGroundTruth_*.csv
    python fit_intrinsics.py data.csv --mount-angle-search
    python fit_intrinsics.py holdout.csv --evaluate

    --mount-angle-search   sweep mountAngle maximising fit linearity. Leave it
                           off while the camera is level; turn it on once the
                           slanted mount is printed.
    --evaluate             skip fitting, just score the FloorErrMM / SizeErrMM
                           columns already in the CSV against the pass criteria.

INPUT
    The CSV written by RaytracingDataCollector. Distances are measured from
    ROBOT CENTER; the camera offsets are carried in every row so this script can
    convert to camera-relative geometry itself.

Standard library only, on purpose -- no numpy, no new dependencies.
"""

import argparse
import csv
import math
import sys
from collections import defaultdict

# Pass criteria from the plan.
TOL_FRAC = 0.05      # +/-5% ...
TOL_FLOOR_MM = 50.0  # ... or +/-50 mm, whichever is larger
TOL_LATERAL_MM = 40.0

CENTERED_LATERAL_MM = 25.0  # rows within this of centre feed the cy/fy fit


# ---------------------------------------------------------------------------
# tiny stats helpers
# ---------------------------------------------------------------------------

def linreg(xs, ys):
    """Ordinary least squares. Returns (slope, intercept, r_squared)."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        return None
    slope = sxy / sxx
    intercept = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 1.0
    return slope, intercept, r2


def mean(vs):
    return sum(vs) / len(vs) if vs else float("nan")


def stdev(vs):
    if len(vs) < 2:
        return float("nan")
    m = mean(vs)
    return math.sqrt(sum((v - m) ** 2 for v in vs) / (len(vs) - 1))


def fnum(s):
    """Parse a float, mapping the Java-side NaN spellings to None."""
    try:
        v = float(s)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(v) or math.isinf(v) else v


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def load(path):
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            try:
                known_dist = fnum(raw["KnownDistMM"])
                known_lat = fnum(raw["KnownLateralMM"])
                if known_dist is None or known_lat is None or known_dist <= 0:
                    continue
                # |lateral| can never exceed the straight-line distance.
                if abs(known_lat) > known_dist:
                    continue

                row = {
                    "known_dist": known_dist,
                    "known_lat": known_lat,
                    "known_fwd": math.sqrt(known_dist ** 2 - known_lat ** 2),
                    "px_bottom_x": fnum(raw["PixelBottomX"]),
                    "px_bottom_y": fnum(raw["PixelBottomY"]),
                    "px_center_x": fnum(raw["PixelCenterX"]),
                    "px_center_y": fnum(raw["PixelCenterY"]),
                    "bbox_w": fnum(raw["BboxW"]),
                    "bbox_h": fnum(raw["BboxH"]),
                    "label": raw.get("Label", "?"),
                    "floor_err": fnum(raw.get("FloorErrMM")),
                    "floor_lat": fnum(raw.get("FloorLateralMM")),
                    "size_err": fnum(raw.get("SizeErrMM")),
                    "size_lat": fnum(raw.get("SizeLateralMM")),
                    "diameter": fnum(raw.get("ObjDiameterMM")) or 127.0,
                    "cam_h": fnum(raw.get("CamHeightMM")) or 236.163,
                    "cam_x": fnum(raw.get("CamOffsetXMM")) or -157.548,
                    "cam_z": fnum(raw.get("CamOffsetZMM")) or 151.868,
                    "shipped_fx": fnum(raw.get("Fx")),
                    "shipped_fy": fnum(raw.get("Fy")),
                    "shipped_cx": fnum(raw.get("Cx")),
                    "shipped_cy": fnum(raw.get("Cy")),
                }
            except KeyError as exc:
                sys.exit("CSV is missing column %s -- is this a "
                         "RaytracingDataCollector file?" % exc)

            # Ball position relative to the CAMERA, in the camera's
            # (lateral, forward) ground frame. +lateral = LEFT, matching
            # CameraIntrinsics' normX = cx - pixelX convention.
            row["dx"] = row["known_lat"] - row["cam_x"]
            row["dz"] = row["known_fwd"] - row["cam_z"]
            if row["dz"] <= 0:
                continue

            # Straight-line camera-to-ball-centre range, for the size fits.
            radius = row["diameter"] / 2.0
            row["range"] = math.sqrt(
                row["dx"] ** 2 + row["dz"] ** 2 + (row["cam_h"] - radius) ** 2)
            rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# fits
# ---------------------------------------------------------------------------

def u_of(dz, cam_h, theta):
    """
    Normalised image-Y ray component for a floor point at forward distance dz,
    given camera height cam_h and downward mount angle theta (radians).

    Derived from CameraIntrinsics.calculateRobotFramePos by solving its floor
    intersection for the ray:
        dz = -h*(u*sin + cos) / (u*cos - sin)
      =>  u = (dz*sin - h*cos) / (dz*cos + h*sin)
    and pixelY = cy - fy*u, which stays LINEAR in the unknowns (cy, fy) for any
    fixed theta -- which is what makes the angle search a simple 1-D sweep.
    """
    s, c = math.sin(theta), math.cos(theta)
    denom = dz * c + cam_h * s
    if abs(denom) < 1e-9:
        return None
    return (dz * s - cam_h * c) / denom


def fit_vertical(rows, theta):
    """Regress pixelY against u(dz) -> intercept = cy, -slope = fy."""
    xs, ys = [], []
    for r in rows:
        if abs(r["known_lat"]) > CENTERED_LATERAL_MM:
            continue
        if r["px_bottom_y"] is None:
            continue
        u = u_of(r["dz"], r["cam_h"], theta)
        if u is None:
            continue
        xs.append(u)
        ys.append(r["px_bottom_y"])
    if len(xs) < 3:
        return None
    fit = linreg(xs, ys)
    if fit is None:
        return None
    slope, intercept, r2 = fit
    return {"fy": -slope, "cy": intercept, "r2": r2, "n": len(xs)}


def fit_from_size(rows, key):
    """Regress bbox dimension against 1/range -> slope = D * focal."""
    xs, ys = [], []
    diameter = None
    for r in rows:
        v = r[key]
        if v is None or v <= 0 or r["range"] <= 0:
            continue
        diameter = r["diameter"]
        xs.append(1.0 / r["range"])
        ys.append(v)
    if len(xs) < 3 or not diameter:
        return None
    fit = linreg(xs, ys)
    if fit is None:
        return None
    slope, intercept, r2 = fit
    return {"focal": slope / diameter, "intercept_px": intercept,
            "r2": r2, "n": len(xs)}


def fit_horizontal(rows):
    """Regress pixelX against dx/dz -> intercept = cx, slope = -fx."""
    xs, ys = [], []
    for r in rows:
        if r["px_center_x"] is None or r["dz"] <= 0:
            continue
        xs.append(r["dx"] / r["dz"])
        ys.append(r["px_center_x"])
    if len(xs) < 3:
        return None
    fit = linreg(xs, ys)
    if fit is None:
        return None
    slope, intercept, r2 = fit
    return {"cx": intercept, "fx": -slope, "r2": r2, "n": len(xs)}


def search_mount_angle(rows):
    """Sweep theta for the best vertical-fit R^2. Coarse pass then fine pass."""
    best = None
    for step, span, centre in ((0.5, 45.0, 0.0), (0.01, 0.6, None)):
        lo = (centre if centre is not None else best["theta_deg"]) - span
        hi = (centre if centre is not None else best["theta_deg"]) + span
        deg = lo
        while deg <= hi:
            res = fit_vertical(rows, math.radians(deg))
            if res and (best is None or res["r2"] > best["r2"]):
                best = dict(res, theta_deg=deg)
            deg += step
    return best


# ---------------------------------------------------------------------------
# residuals and reporting
# ---------------------------------------------------------------------------

def vertical_residuals(rows, fy, cy, theta):
    """Back-project each sample with the fitted constants; error in mm."""
    out = []
    for r in rows:
        if abs(r["known_lat"]) > CENTERED_LATERAL_MM or r["px_bottom_y"] is None:
            continue
        yd = (cy - r["px_bottom_y"]) / fy
        s, c = math.sin(theta), math.cos(theta)
        world_y = yd * c - s
        world_z = yd * s + c
        if world_y > -1e-6:
            out.append((r["dz"], None))
            continue
        dz_hat = (-r["cam_h"] / world_y) * world_z
        out.append((r["dz"], dz_hat - r["dz"]))
    return out


def report_fit(rows, args):
    theta_deg = 0.0
    if args.mount_angle_search:
        best = search_mount_angle(rows)
        if not best:
            sys.exit("Not enough centred samples to search the mount angle.")
        theta_deg = best["theta_deg"]
        print("MOUNT ANGLE SEARCH")
        print("  best theta      : %+.2f deg  (R^2 %.5f)" % (theta_deg, best["r2"]))
        print("  positive theta means the lens points DOWN.")
        print()

    theta = math.radians(theta_deg)

    vert = fit_vertical(rows, theta)
    if not vert:
        sys.exit("Need at least 3 centred samples (|lateral| <= %.0f mm) with a "
                 "bottom pixel to fit cy/fy." % CENTERED_LATERAL_MM)

    horiz = fit_horizontal(rows)
    size_w = fit_from_size(rows, "bbox_w")
    size_h = fit_from_size(rows, "bbox_h")

    shipped_fx = rows[0]["shipped_fx"]
    shipped_fy = rows[0]["shipped_fy"]
    shipped_cx = rows[0]["shipped_cx"]
    shipped_cy = rows[0]["shipped_cy"]

    def cmp_line(name, fitted, shipped):
        if fitted is None:
            return "  %-4s fitted    n/a" % name
        if shipped is None:
            return "  %-4s fitted %10.3f" % (name, fitted)
        delta = fitted - shipped
        pct = (100.0 * delta / shipped) if shipped else float("nan")
        return ("  %-4s fitted %10.3f   shipped %10.3f   delta %+8.3f (%+6.2f%%)"
                % (name, fitted, shipped, delta, pct))

    print("=" * 78)
    print("FITTED INTRINSICS   (%d rows loaded)" % len(rows))
    print("=" * 78)
    print("Vertical fit  pixelY vs u(dz)      n=%d  R^2=%.5f"
          % (vert["n"], vert["r2"]))
    print(cmp_line("fy", vert["fy"], shipped_fy))
    print(cmp_line("cy", vert["cy"], shipped_cy))
    print()

    if horiz:
        print("Horizontal fit  pixelX vs dx/dz   n=%d  R^2=%.5f"
              % (horiz["n"], horiz["r2"]))
        print(cmp_line("cx", horiz["cx"], shipped_cx))
        print(cmp_line("fx", horiz["fx"], shipped_fx))
    else:
        print("Horizontal fit  SKIPPED - need off-centre samples to solve cx/fx.")
        print("  Collect ~6 rows with the ball off to one side at 2-3 distances.")
    print()

    if size_w:
        print("Size fit  bboxW vs 1/range        n=%d  R^2=%.5f"
              % (size_w["n"], size_w["r2"]))
        print(cmp_line("fx", size_w["focal"], shipped_fx))
    if size_h:
        print("Size fit  bboxH vs 1/range        n=%d  R^2=%.5f"
              % (size_h["n"], size_h["r2"]))
        print(cmp_line("fy", size_h["focal"], shipped_fy))
    print()

    # --- the R1 verdict -----------------------------------------------------
    aspects = [r["bbox_w"] / r["bbox_h"] for r in rows
               if r["bbox_w"] and r["bbox_h"] and r["bbox_h"] > 0]
    print("-" * 78)
    print("ASPECT CHECK  (the cheap verdict on the 0.5/0.6 rescale)")
    if aspects:
        observed = mean(aspects)
        print("  observed mean bbox w/h : %.4f  (sd %.4f, n=%d)"
              % (observed, stdev(aspects), len(aspects)))
        if shipped_fx and shipped_fy:
            print("  shipped fx/fy          : %.4f" % (shipped_fx / shipped_fy))
        print("  1.0000 would mean square pixels, i.e. fy is wrong by ~fx/fy.")
        if abs(observed - 1.0) < 0.05:
            print("  VERDICT: reads ~1.0 -> the 0.5/0.6 rescale is WRONG.")
            print("           fx and fy should be equal. Use the fitted values.")
        elif shipped_fx and shipped_fy and \
                abs(observed - shipped_fx / shipped_fy) < 0.05:
            print("  VERDICT: matches shipped fx/fy -> the rescale is correct.")
        else:
            print("  VERDICT: matches neither. Trust the fitted constants below.")
    else:
        print("  no usable bbox rows.")
    print()

    # --- residuals ----------------------------------------------------------
    res = vertical_residuals(rows, vert["fy"], vert["cy"], theta)
    good = [(dz, e) for dz, e in res if e is not None]
    rejected = len(res) - len(good)
    print("-" * 78)
    print("RESIDUALS after fit  (floor projection, centred samples)")
    if good:
        errs = [e for _, e in good]
        print("  mean %+.1f mm   sd %.1f mm   worst %+.1f mm   n=%d"
              % (mean(errs), stdev(errs), max(errs, key=abs), len(errs)))
        by_dist = defaultdict(list)
        for dz, e in good:
            by_dist[round(dz / 250.0) * 250].append(e)
        print("  %-12s %8s %8s %6s" % ("dz bucket", "mean", "sd", "n"))
        for bucket in sorted(by_dist):
            es = by_dist[bucket]
            print("  %-12s %+8.1f %8.1f %6d"
                  % ("%d mm" % bucket, mean(es), stdev(es), len(es)))
    if rejected:
        print("  %d centred samples projected ABOVE the horizon (no solution)."
              % rejected)
    print()

    # --- paste-ready --------------------------------------------------------
    fx_final = None
    for candidate in (horiz["fx"] if horiz else None,
                      size_w["focal"] if size_w else None):
        if candidate:
            fx_final = candidate
            break
    cx_final = horiz["cx"] if horiz else shipped_cx

    print("=" * 78)
    print("PASTE INTO CameraIntrinsics.java")
    print("=" * 78)
    if fx_final is None or cx_final is None:
        print("  (fx/cx not solved - collect off-centre samples, then re-run.)")
        fx_final = fx_final or shipped_fx
        cx_final = cx_final or shipped_cx
    print("""    public static final CameraIntrinsics ARDUCAM = new CameraIntrinsics(
            %.5f, %.5f,
            %.5f, %.5f,
            0.045011, -0.059862, 0.000330,
            0.001499, 0.005590,
            Math.toRadians(%.3f),
            new Vector3d(%.3f, %.3f, %.3f) // offsets
    );""" % (fx_final, vert["fy"], cx_final, vert["cy"], theta_deg,
             rows[0]["cam_x"], rows[0]["cam_h"], rows[0]["cam_z"]))
    print()
    print("Also update ArtifactDetectionTest.EXPECTED_ASPECT to %.3f"
          % (fx_final / vert["fy"]))


def report_evaluate(rows):
    """Score a hold-out CSV against the plan's pass criteria."""
    print("=" * 78)
    print("HOLD-OUT EVALUATION   (%d rows)" % len(rows))
    print("=" * 78)
    print("PASS = |err| within max(5% of known, 50 mm), lateral within 40 mm")
    print()

    by_dist = defaultdict(list)
    for r in rows:
        by_dist[r["known_dist"]].append(r)

    header = ("  %-10s %5s | %9s %9s %6s | %9s %9s %6s"
              % ("known", "n", "floor mn", "floor sd", "pass",
                 "size mn", "size sd", "pass"))
    print(header)
    print("  " + "-" * (len(header) - 2))

    floor_all_pass = True
    size_all_pass = True

    for dist in sorted(by_dist):
        group = by_dist[dist]
        tol = max(TOL_FRAC * dist, TOL_FLOOR_MM)

        f_errs = [r["floor_err"] for r in group if r["floor_err"] is not None]
        s_errs = [r["size_err"] for r in group if r["size_err"] is not None]

        def score(errs, lat_key):
            if not errs:
                return "NONE", False
            ok_dist = abs(mean(errs)) <= tol
            lats = [r[lat_key] for r in group
                    if r[lat_key] is not None]
            ok_lat = (not lats) or all(
                abs(l - r["known_lat"]) <= TOL_LATERAL_MM
                for l, r in zip(lats, group))
            good = ok_dist and ok_lat
            return ("PASS" if good else "FAIL"), good

        f_mark, f_ok = score(f_errs, "floor_lat")
        s_mark, s_ok = score(s_errs, "size_lat")
        floor_all_pass &= f_ok
        size_all_pass &= s_ok

        print("  %-10.0f %5d | %+9.1f %9.1f %6s | %+9.1f %9.1f %6s"
              % (dist, len(group),
                 mean(f_errs) if f_errs else float("nan"),
                 stdev(f_errs) if f_errs else float("nan"), f_mark,
                 mean(s_errs) if s_errs else float("nan"),
                 stdev(s_errs) if s_errs else float("nan"), s_mark))

        if f_errs and len(f_errs) < len(group):
            print("           %d/%d rows had NO floor solution (above horizon)"
                  % (len(group) - len(f_errs), len(group)))

    print()
    print("  FLOOR PROJECTION : %s" % ("PASS" if floor_all_pass else "FAIL"))
    print("  SIZE-BASED       : %s" % ("PASS" if size_all_pass else "FAIL"))
    print()

    # Where does each method win? That is the crossover to wire into BlobUtils.
    print("-" * 78)
    print("CROSSOVER  (which method is more accurate per distance)")
    crossover = None
    for dist in sorted(by_dist):
        group = by_dist[dist]
        f = [abs(r["floor_err"]) for r in group if r["floor_err"] is not None]
        s = [abs(r["size_err"]) for r in group if r["size_err"] is not None]
        if not f and not s:
            continue
        fm = mean(f) if f else float("inf")
        sm = mean(s) if s else float("inf")
        winner = "floor" if fm < sm else "size"
        if winner == "size" and crossover is None:
            crossover = dist
        print("  %6.0f mm : floor %8.1f   size %8.1f   -> %s"
              % (dist, fm, sm, winner))
    print()
    tested = sorted(by_dist)
    if crossover is None:
        print("  Floor projection won at every distance tested -- keep using it.")
    elif tested and crossover == tested[0]:
        print("  Size-based won at EVERY distance tested (from %.0f mm up)."
              % crossover)
        print("  There is no crossover in this range: prefer size-based ranging,")
        print("  and re-check once the slanted mount moves the horizon.")
    else:
        print("  Crossover at %.0f mm: floor projection is better below it,"
              % crossover)
        print("  size-based is better above it. Wire that split into")
        print("  BlobUtils.findClosestToRobotWorld.")


# ---------------------------------------------------------------------------
# self-check
# ---------------------------------------------------------------------------

def selftest():
    """
    Generate samples from KNOWN intrinsics, then confirm the fits recover them.

    This is the only thing standing between "the regression ran" and "the
    regression is right", so run it after touching any of the math above:
        python fit_intrinsics.py --selftest
    """
    truth_fx, truth_fy = 444.14195, 532.27560
    truth_cx, truth_cy = 350.01860, 212.43984
    cam_h, cam_x, cam_z, diameter = 236.163, -157.548, 151.868, 127.0
    radius = diameter / 2.0

    def synth(theta_deg, fy_override=None):
        theta = math.radians(theta_deg)
        fy = fy_override if fy_override else truth_fy
        out = []
        plan = [(d, 0.0) for d in (400, 500, 600, 750, 900, 1100,
                                   1400, 1700, 2100, 2500)]
        plan += [(d, lat) for d in (700, 1200, 1800) for lat in (-300, 300)]
        for dist, lat in plan:
            fwd = math.sqrt(dist ** 2 - lat ** 2)
            dx, dz = lat - cam_x, fwd - cam_z
            if dz <= 0:
                continue
            u = u_of(dz, cam_h, theta)
            rng = math.sqrt(dx ** 2 + dz ** 2 + (cam_h - radius) ** 2)
            out.append({
                "known_dist": dist, "known_lat": lat, "known_fwd": fwd,
                "dx": dx, "dz": dz, "range": rng,
                "px_bottom_y": truth_cy - fy * u,
                "px_center_x": truth_cx - truth_fx * dx / dz,
                "bbox_w": diameter * truth_fx / rng,
                "bbox_h": diameter * fy / rng,
                "diameter": diameter, "cam_h": cam_h,
                "cam_x": cam_x, "cam_z": cam_z,
                "shipped_fx": truth_fx, "shipped_fy": truth_fy,
                "shipped_cx": truth_cx, "shipped_cy": truth_cy,
                "floor_err": None, "size_err": None,
                "floor_lat": None, "size_lat": None,
            })
        return out

    failures = []

    def check(name, got, want, tol):
        ok = got is not None and abs(got - want) <= tol
        print("  %-34s %12.5f  want %12.5f  %s"
              % (name, got if got is not None else float("nan"), want,
                 "ok" if ok else "FAIL"))
        if not ok:
            failures.append(name)

    print("SELF-CHECK: recover known intrinsics from synthetic samples")
    print()
    print("level camera (mountAngle = 0)")
    rows = synth(0.0)
    vert = fit_vertical(rows, 0.0)
    horiz = fit_horizontal(rows)
    size_w = fit_from_size(rows, "bbox_w")
    size_h = fit_from_size(rows, "bbox_h")
    check("fy from vertical fit", vert and vert["fy"], truth_fy, 0.01)
    check("cy from vertical fit", vert and vert["cy"], truth_cy, 0.01)
    check("fx from horizontal fit", horiz and horiz["fx"], truth_fx, 0.01)
    check("cx from horizontal fit", horiz and horiz["cx"], truth_cx, 0.01)
    check("fx from bbox width", size_w and size_w["focal"], truth_fx, 0.5)
    check("fy from bbox height", size_h and size_h["focal"], truth_fy, 0.5)

    print()
    print("tilted camera (mountAngle = 18 deg), recovered by search")
    rows = synth(18.0)
    best = search_mount_angle(rows)
    check("mount angle (deg)", best and best["theta_deg"], 18.0, 0.02)
    check("fy at fitted angle", best and best["fy"], truth_fy, 0.5)
    check("cy at fitted angle", best and best["cy"], truth_cy, 0.5)

    print()
    print("square-pixel case: TRUE fy == fx, shipped fy is wrong by 19%")
    rows = synth(0.0, fy_override=truth_fx)
    vert = fit_vertical(rows, 0.0)
    check("fy recovered as fx", vert and vert["fy"], truth_fx, 0.01)
    aspects = [r["bbox_w"] / r["bbox_h"] for r in rows]
    check("bbox aspect reads 1.0", mean(aspects), 1.0, 0.001)

    print()
    if failures:
        print("SELF-CHECK FAILED: %s" % ", ".join(failures))
        return 1
    print("SELF-CHECK PASSED")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Fit Arducam intrinsics from RaytracingDataCollector CSV.")
    parser.add_argument("csv", nargs="?", help="RaytracingGroundTruth_*.csv")
    parser.add_argument("--mount-angle-search", action="store_true",
                        help="sweep mountAngle for best linearity "
                             "(use once the slanted mount is installed)")
    parser.add_argument("--evaluate", action="store_true",
                        help="score a hold-out CSV instead of fitting")
    parser.add_argument("--selftest", action="store_true",
                        help="verify the fits against synthetic known-truth data")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.csv:
        parser.error("a CSV path is required (or use --selftest)")

    rows = load(args.csv)
    if not rows:
        sys.exit("No usable rows in %s" % args.csv)

    if args.evaluate:
        report_evaluate(rows)
    else:
        report_fit(rows, args)


if __name__ == "__main__":
    main()
