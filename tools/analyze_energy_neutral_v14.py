"""Frozen-pose CAD potential diagnostics; never writes firmware or controller gains.

All mass is provisionally lumped at a body-fixed CG. A contact-switch angle is
compared with the minimum of the resulting CG-height landscape, not equated to
it. Absolute camera-to-CAD offsets, moving leg masses and actual contact are not
identified by this calculation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(r"C:\Users\arika\Downloads\RW_Walker_Dynamics\rw_dynamics_package\01_CAD\rw_walker_uploaded_assembly.glb")
EXPECTED = "63be3da5c06b9f198a3d6409b4b0a68ead9c2a638d87f06e7914c40b13693884"
MASS, GRAVITY = .1997, 9.80665


def load_feet(source: Path) -> tuple[dict, np.ndarray, str]:
    raw = source.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    assert sha == EXPECTED, "Unexpected CAD source; audit geometry before reuse"
    length, kind = struct.unpack_from("<II", raw, 12)
    assert kind == 0x4e4f534a
    doc = json.loads(raw[20:20 + length])
    size, kind = struct.unpack_from("<II", raw, 20 + length)
    assert kind == 0x004e4942 and len(doc["meshes"]) == 18
    blob = raw[28 + length:28 + length + size]
    meshes = {}
    for node in doc["nodes"]:
        if "mesh" not in node:
            continue
        assert not any(k in node for k in ("matrix", "translation", "rotation", "scale"))
        mesh = doc["meshes"][node["mesh"]]
        a = doc["accessors"][mesh["primitives"][0]["attributes"]["POSITION"]]
        v = doc["bufferViews"][a["bufferView"]]
        assert a["componentType"] == 5126 and a["type"] == "VEC3" and v.get("byteStride", 12) == 12
        points = np.frombuffer(blob, dtype="<f4", offset=v.get("byteOffset", 0) + a.get("byteOffset", 0), count=a["count"] * 3).reshape(-1, 3).astype(float)
        meshes[node["name"]] = np.unique(points, axis=0)
    leg = meshes["CAD_solid_00"]
    d = np.linalg.norm(leg[:, 1:] - [-17.579, 82.825], axis=1)
    ring = np.unique(leg[(d > 7.98) & (d < 8.02), 1:], axis=0)
    c = np.linalg.lstsq(np.c_[2 * ring[:, 0], 2 * ring[:, 1], np.ones(len(ring))], np.sum(ring * ring, axis=1), rcond=None)[0]
    assert len(ring) > 20 and abs(math.sqrt(c[2] + c[0] ** 2 + c[1] ** 2) - 8) < .02
    hip = np.array([0., c[0], c[1]])
    feet = {"right": meshes["CAD_solid_05"] - hip, "left": meshes["CAD_solid_07"] - hip}
    assert feet["left"][:, 0].mean() < 0 < feet["right"][:, 0].mean()
    return feet, hip, sha


def low_height(points: np.ndarray, foot: float, pitch: float, angles: np.ndarray) -> np.ndarray:
    # R_y(roll) R_x(pitch) R_x(foot); dimensions are mm.
    a = math.radians(foot + pitch)
    z = math.sin(a) * points[:, 1] + math.cos(a) * points[:, 2]
    theta = np.radians(np.atleast_1d(angles))
    return np.min(-np.sin(theta)[:, None] * points[:, 0] + np.cos(theta)[:, None] * z, axis=1)


def reconstruct_sole_arcs(feet: dict) -> tuple[dict, dict]:
    """Reconstruct the exact circle hidden by four coarse GLB sole chords.

    This is explicitly an additional surface-model assumption, not an edit of
    the CAD source. Retain all original vertices and add the inferred arc.
    """
    smoothed, models = {}, {}
    for side, points in feet.items():
        groups = {}
        for v in points:
            key = round(float(v[0]), 4)
            if key not in groups or v[2] < groups[key][2]:
                groups[key] = v
        profile = sorted((v[[0, 2]] for v in groups.values()), key=lambda v: v[0])
        hull = []
        for v in profile:
            while len(hull) >= 2:
                a, b = hull[-2], hull[-1]
                cross = (b[0] - a[0]) * (v[1] - a[1]) - (b[1] - a[1]) * (v[0] - a[0])
                if cross > 0:
                    break
                hull.pop()
            hull.append(v)
        hull = np.array(hull)
        c = np.linalg.lstsq(np.c_[2 * hull[:, 0], 2 * hull[:, 1], np.ones(len(hull))], np.sum(hull * hull, axis=1), rcond=None)[0]
        radius = math.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)
        residual = float(np.max(np.abs(np.linalg.norm(hull - c[:2], axis=1) - radius)))
        assert len(hull) == 4 and abs(radius - 150) < .02 and residual < .001
        x = np.linspace(hull[0, 0], hull[-1, 0], 801)
        z = c[1] - np.sqrt(radius ** 2 - (x - c[0]) ** 2)
        added = [np.c_[x, np.full(len(x), y), z] for y in (points[:, 1].min(), points[:, 1].max())]
        smoothed[side] = np.vstack([points, *added])
        models[side] = dict(profile_points_mm=hull.tolist(), center_x_mm=float(c[0]), center_z_mm=float(c[1]),
                            radius_mm=radius, maximum_circle_residual_mm=residual,
                            added_arc_samples=801, y_extrema_mm=[float(points[:, 1].min()), float(points[:, 1].max())])
    return smoothed, models


def landscape(feet: dict, angles: np.ndarray, left: float, right: float, pitch: float,
              height_mm: float = 120., cg_x_mm: float = 0., cg_y_mm: float = 0.) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ll = low_height(feet["left"], left, pitch, angles)
    rr = low_height(feet["right"], right, pitch, angles)
    support = -np.minimum(ll, rr)
    neutral_support = -min(low_height(feet["left"], 0, 0, np.array([0]))[0], low_height(feet["right"], 0, 0, np.array([0]))[0])
    cg_z_mm = height_mm - neutral_support
    t = np.radians(np.atleast_1d(angles))
    p = math.radians(pitch)
    body_cg_z = -np.sin(t) * cg_x_mm + np.cos(t) * (math.sin(p) * cg_y_mm + math.cos(p) * cg_z_mm)
    return support + body_cg_z, ll - rr, support


def golden_min(f, lo: float, hi: float) -> float:
    ratio = (math.sqrt(5) - 1) / 2
    x1, x2 = hi - ratio * (hi - lo), lo + ratio * (hi - lo)
    y1, y2 = f(x1), f(x2)
    for _ in range(50):
        if y1 < y2:
            hi, x2, y2 = x2, x1, y1
            x1 = hi - ratio * (hi - lo)
            y1 = f(x1)
        else:
            lo, x1, y1 = x1, x2, y2
            x2 = lo + ratio * (hi - lo)
            y2 = f(x2)
    return (lo + hi) / 2


def summary(feet: dict, angles: np.ndarray, case: dict, height: float = 120., cg_x: float = 0., cg_y: float = 0.) -> tuple[dict, np.ndarray]:
    l, r, p = case["left_deg"], case["right_deg"], case["pitch_deg"]
    z, gap, support = landscape(feet, angles, l, r, p, height, cg_x, cg_y)
    i = int(np.argmin(z))
    f = lambda a: float(landscape(feet, np.array([a]), l, r, p, height, cg_x, cg_y)[0][0])
    minimum = golden_min(f, angles[max(0, i - 1)], angles[min(len(angles) - 1, i + 1)])
    lo, hi = -15., 15.
    fl = float(landscape(feet, np.array([lo]), l, r, p, height, cg_x, cg_y)[1][0])
    fh = float(landscape(feet, np.array([hi]), l, r, p, height, cg_x, cg_y)[1][0])
    switch = None
    if fl * fh <= 0:
        for _ in range(45):
            mid = (lo + hi) / 2
            fm = float(landscape(feet, np.array([mid]), l, r, p, height, cg_x, cg_y)[1][0])
            if fl * fm <= 0:
                hi = mid
            else:
                lo, fl = mid, fm
        switch = (lo + hi) / 2
    zero_z, min_z = f(0), f(minimum)
    e = MASS * GRAVITY * (z - height)  # mm -> m and J -> mJ cancel.
    out = dict(**case, height_mm=height, cg_x_mm=cg_x, cg_y_mm=cg_y,
               energy_minimum_deg=minimum, minimum_at_scan_boundary=i in (0, len(angles) - 1),
               equal_support_deg=switch, energy_minimum_minus_support_switch_deg=None if switch is None else minimum - switch,
               cg_height_at_minimum_mm=min_z, cg_height_at_mechanical_zero_mm=zero_z,
               potential_rise_minimum_to_mechanical_zero_mj=MASS * GRAVITY * (zero_z - min_z),
               support_height_at_zero_mm=float(landscape(feet, np.array([0]), l, r, p, height, cg_x, cg_y)[2][0]))
    return out, e


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=ROOT / "analysis/2026-11-13-walking/v14_energy_neutral")
    args = parser.parse_args()
    feet, hip, sha = load_feet(args.source)
    smooth_feet, arc_models = reconstruct_sole_arcs(feet)
    angles = np.linspace(-12, 12, 2401)
    cases = [dict(name="neutral", left_deg=0, right_deg=0, pitch_deg=0),
             *[dict(name=f"right_{r}", left_deg=0, right_deg=r, pitch_deg=0) for r in (1, 2, 4, 6, 8)],
             dict(name="left_4", left_deg=4, right_deg=0, pitch_deg=0),
             dict(name="both_4", left_deg=4, right_deg=4, pitch_deg=0),
             dict(name="camera_pose_hypothesis", left_deg=-.80957, right_deg=3.74576, pitch_deg=0)]
    results, raw_results, curves, raw_curves = [], [], {}, {}
    for case in cases:
        result, curve = summary(smooth_feet, angles, case)
        results.append(result)
        curves[case["name"]] = curve
        result, curve = summary(feet, angles, case)
        raw_results.append(result)
        raw_curves[case["name"]] = curve
    sensitivity = []
    for pitch in (-2, 0, 2):
        for height in (100, 120, 140):
            for cg_x in (-2, 0, 2):
                case = dict(name="right_4_sensitivity", left_deg=0, right_deg=4, pitch_deg=pitch)
                result, _ = summary(smooth_feet, angles, case, height, cg_x)
                sensitivity.append(result)
    report = dict(source=str(args.source), source_sha256=sha, hip_mm=hip.tolist(), mass_kg=MASS,
                  gravity_m_s2=GRAVITY, units="angles_deg;heights_mm;energy_mJ", scan_roll_deg=[-12, 12],
                  formula="z_CG=-min(foot_vertex_z)+[Ry(roll)Rx(pitch)c_body]_z; c_body.z=H-support(neutral); c_body.x=c_body.y=0 unless sensitivity says otherwise",
                  energy_reference="fixed H at neutral pose: E_mJ=mass*g*(z_CG_mm-H_mm)",
                  surface_model="circular sole arcs reconstructed from four lower-hull points per foot; raw mesh diagnostics stored separately",
                  arc_models=arc_models, results=results, raw_mesh_results=raw_results,
                  raw_mesh_maximum_baseline_error_mj=float(np.max(np.abs(raw_curves['neutral']-curves['neutral']))), sensitivity=sensitivity,
                  limits=["all mass lumped at body-fixed CG; moving leg/foot masses not identified", "frozen foot poses and rigid level floor; no friction, impacts or foot reaction", "forward-positive camera sign known, absolute camera-to-CAD zeros still uncalibrated", "equal-support point and potential minimum are different definitions", "not active control; nominal passive fit cannot be corrected by a simple coordinate shift without reidentification"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for name in ("neutral", "right_2", "right_4", "right_6", "left_4"):
        row = next(r for r in results if r["name"] == name)
        curve = curves[name]
        # Subtract each frozen-pose minimum ONLY for visual comparison; not a
        # valid pose-dependent global energy gauge for time-varying feet.
        axes[0].plot(angles, curve - min(curve), label=name)
        axes[0].plot(row["energy_minimum_deg"], 0, "o", ms=4)
    axes[0].set(xlabel="CAD body roll [deg]; MEKF sign/zero uncalibrated", ylabel="Frozen-pose U minus its minimum [mJ]", xlim=(-10, 10), ylim=(-.05, 6))
    axes[0].axvline(0, c="k", lw=.6, label="CAD mechanical zero")
    axes[0].legend(fontsize=8)
    rows = results[:6]
    axes[1].plot([r["right_deg"] for r in rows], [r["equal_support_deg"] for r in rows], "o-", label="Equal foot support heights")
    axes[1].plot([r["right_deg"] for r in rows], [r["energy_minimum_deg"] for r in rows], "o-", label="Lumped-CG potential minimum")
    axes[1].axhline(0, c="k", lw=.6)
    axes[1].set(xlabel="Frozen right-foot CAD angle [deg], left=0", ylabel="CAD body roll [deg]; MEKF sign/zero uncalibrated")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Frozen-pose valley shapes: fixed CG H=120 mm; each U-min subtracted for plotting, not cross-time energy")
    fig.savefig(args.output.with_suffix(".png"), dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    for name in ("neutral", "right_1", "right_4"):
        ax.plot(angles, curves[name]-min(curves[name]), label=name+" reconstructed arc")
        ax.plot(angles, raw_curves[name]-min(raw_curves[name]), "--", alpha=.7, label=name+" raw four-chord mesh")
    ax.axvline(0, c="k", lw=.6)
    ax.set(xlabel="CAD body roll [deg]; MEKF sign/zero uncalibrated", ylabel="Frozen-pose U minus its minimum [mJ]", xlim=(-10, 10), ylim=(-.02, 1.5))
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
    fig.suptitle("Mesh tessellation creates spurious wells; raw CAD vertices are not an exact rolling surface")
    fig.savefig(args.output.with_name(args.output.name+"_mesh_comparison").with_suffix(".png"), dpi=160)
    plt.close(fig)
    print(json.dumps(dict(results=results, sensitivity_minimum_range_deg=[min(r["energy_minimum_deg"] for r in sensitivity), max(r["energy_minimum_deg"] for r in sensitivity)]), indent=2))


if __name__ == "__main__":
    main()
