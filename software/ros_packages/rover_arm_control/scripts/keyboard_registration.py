"""
Point cloud to CAD registration of a tenkeyless keyboard scanned with a RealSense D405.

Follows the welding perception pipeline (Test_open3d_v3 / full_pipeline):
    1. Load capture -> point cloud (mm)
    2. Segment the object from the scene (plane removal, outlier removal, DBSCAN)
    3. Preprocess (voxel downsample, normals, FPFH)
    4. Parallel RANSAC global registration attempts
    5. Multi-scale point-to-plane ICP on every global result
    6. Visualize and report alignment statistics

Differences from the Zivid welding captures:
    - The D405 capture is a raw Z16 depth image, so it is deprojected with the intrinsics in the
      metadata csv. D405 depth units are 0.1 mm.
    - The scene needs pre-processing: desk plane, cable, mouse and depth noise are removed and the
      keyboard is kept as the largest cluster.
    - The camera only sees the keyboard from above, so the CAD model is sampled from the surfaces
      that face the camera (keycap tops and walls) instead of the whole mesh.
    - A keyboard is a repeating grid of identical keys. FPFH + RANSAC seeds land whole key pitches
      off or flipped 180 deg, and at this range the D405 blurs the key gaps so the scan is close to a
      flat slab: ICP fitness scores those wrong poses as high as the right one. Two additions fix this:
        * A 2D template match of the CAD keycap layout against the scan height map adds global seeds.
        * Candidates are ranked by keycap relief (how raised the scan is under the CAD keycaps
          relative to its surroundings) instead of ICP fitness.
      ICP also runs only on scan points over the CAD keycaps, since the case/bezel is not in the CAD.

All geometry is processed in millimetres. Run from anywhere:
    python3 keyboard_registration.py                                # interactive Open3D windows
    python3 keyboard_registration.py --no-vis --save-dir /tmp/kb    # headless, save renders + stats
"""
import argparse
import copy
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import open3d as o3d
from scipy.ndimage import map_coordinates, uniform_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import Test_open3d_v3 as test_o3d  # noqa: E402

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

SCAN_COLOR = [1, 0.706, 0]
CAD_COLOR = [0, 0.651, 0.929]


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def read_realsense_metadata(path):
    """Parse the RealSense Viewer *_metadata.csv for resolution and intrinsics."""
    values = {}
    with open(path, "r") as f:
        for line in f:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 2 and parts[1]:
                values[parts[0]] = parts[1]
    return {
        "width": int(values["Resolution x"]),
        "height": int(values["Resolution y"]),
        "fx": float(values["Fx"]),
        "fy": float(values["Fy"]),
        "ppx": float(values["PPx"]),
        "ppy": float(values["PPy"]),
    }


def load_d405_depth_as_point_cloud(raw_path, meta_path, depth_unit_mm=0.1, max_depth_mm=400.0):
    """Deproject a raw Z16 depth image into a point cloud in the camera optical frame (mm).

    Returns the point cloud, the depth image in mm and the intrinsics.
    """
    meta = read_realsense_metadata(meta_path)
    depth = np.fromfile(raw_path, dtype=np.uint16).reshape(meta["height"], meta["width"])
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        meta["width"], meta["height"], meta["fx"], meta["fy"], meta["ppx"], meta["ppy"])

    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        o3d.geometry.Image(depth), intrinsic,
        depth_scale=1.0 / depth_unit_mm,  # raw units per mm
        depth_trunc=max_depth_mm)
    return pcd, depth.astype(np.float64) * depth_unit_mm, meta


def load_visible_cad(path, n_points=300000, include_walls=True):
    """Sample the CAD surfaces the camera can see from above.

    The keyboard STL is keycaps only, with keycap tops at z=0 facing +z. Every top face is coplanar,
    so a top-only target gives point-to-plane ICP no constraint in x, y or yaw. The keycap walls add
    in-plane normals that lock the key grid. Downward-facing faces are never seen and are dropped.

    Returns the full mesh, the target cloud used for registration and a top-face-only cloud used
    for seeding, cropping, scoring and statistics.
    """
    mesh = o3d.io.read_triangle_mesh(path)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    normals_z = np.asarray(mesh.triangle_normals)[:, 2]

    def sample(mask, n):
        sub = copy.deepcopy(mesh)
        sub.remove_triangles_by_mask(~mask)
        sub.remove_unreferenced_vertices()
        return sub.sample_points_uniformly(n, use_triangle_normal=True)

    top = sample(normals_z > 0.5, n_points)
    target = sample(normals_z > -0.5, n_points) if include_walls else copy.deepcopy(top)
    return mesh, target, top


# ---------------------------------------------------------------------------
# Segmentation
# ---------------------------------------------------------------------------
def segment_keyboard(scene, voxel_size=1.0, plane_dist=4.0, eps=4.0, min_points=30):
    """Remove the desk plane and noise, then keep the largest cluster (the keyboard)."""
    scene_down = scene.voxel_down_sample(voxel_size)
    clusters = test_o3d.segment_welding_object(
        scene_down, visualize=False, eps=eps, min_points=min_points,
        dist_threshold=plane_dist, ransac_n=3, num_iterations=2000,
        nb_neighbors=30, std_ratio=1.5, remove_plane=True, cluster=True)
    keyboard = max(clusters, key=lambda c: len(c.points))
    return keyboard, scene_down


# ---------------------------------------------------------------------------
# Global registration
# ---------------------------------------------------------------------------
def global_registration(scan, cad, voxel_size, num_attempts, max_workers):
    """FPFH + RANSAC global registration, repeated in parallel like the welding pipeline."""
    scan = copy.deepcopy(scan)
    cad = copy.deepcopy(cad)
    # Scan normals must point toward the camera so FPFH signs match the CAD (+z toward camera)
    scan.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    scan.orient_normals_towards_camera_location(np.zeros(3))

    scan_down, scan_fpfh, _ = test_o3d.preprocess_point_cloud(scan, voxel_size)
    cad_down, cad_fpfh, _ = test_o3d.preprocess_point_cloud(cad, voxel_size)
    print(f"Scan downsampled to {len(scan_down.points)} points, CAD to {len(cad_down.points)} points.")

    print(f"Running {num_attempts} global registration attempts on {max_workers} workers...")
    start = time.time()
    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        results = list(ex.map(
            test_o3d.run_parallel_registration,
            range(num_attempts),
            [[scan]] * num_attempts,
            [[scan_down]] * num_attempts,
            [[scan_fpfh]] * num_attempts,
            [cad_down] * num_attempts,
            [cad_fpfh] * num_attempts,
            [voxel_size] * num_attempts,
            [False] * num_attempts,  # rigid: the D405 depth is metric, no scale estimation
        ))
    print(f"Global registration took {time.time() - start:.1f} s")

    # Fitness at exactly 0 or 1 means the global registration failed.
    return [{"source": f"ransac {i}", "global_score": res.fitness, "transformation": np.asarray(res.transformation)}
            for i, res in enumerate(results)
            if abs(res.fitness - 1.0) > 1e-9 and abs(res.fitness) > 1e-9]


def keyboard_frame(scan):
    """Frame on the scanned keyboard: x along the long side (camera +x), z toward the camera."""
    obb = scan.get_minimal_oriented_bounding_box()
    axes = np.asarray(obb.R)[:, np.argsort(-np.asarray(obb.extent))].copy()
    if axes[2, 2] > 0:
        axes[:, 2] *= -1
    if axes[0, 0] < 0:
        axes[:, 0] *= -1
    axes[:, 1] = np.cross(axes[:, 2], axes[:, 0])
    return axes, np.asarray(obb.center)


def template_match_registration(scan, cad_top, res=2.0, yaw_range=4.0, yaw_step=0.5, blur_mm=0.75):
    """2D template match of the CAD keycap layout against the scanned keyboard height map.

    Every key looks the same to FPFH, but the keycap layout as a whole is unique, so correlating a
    high-passed height map of the scan with a keycap mask finds the in-plane pose directly. Returns
    the best seed for each 180 deg flip as scan -> CAD transforms with their correlation scores.
    """
    from scipy.interpolate import griddata
    from scipy.ndimage import gaussian_filter
    from scipy.signal import fftconvolve

    axes, center = keyboard_frame(scan)
    q = (np.asarray(scan.points) - center) @ axes
    qx_min, qy_max = q[:, 0].min(), q[:, 1].max()

    # Scan height map in the keyboard frame (col -> +x, row -> -y), high-passed to keep key edges
    gx, gy = np.meshgrid(np.arange(qx_min, q[:, 0].max(), 1 / res), np.arange(qy_max, q[:, 1].min(), -1 / res))
    height = griddata(q[:, :2], q[:, 2], (gx, gy), method="linear")
    filled = np.nan_to_num(height, nan=np.nanmin(height))
    scan_map = np.where(np.isnan(height), 0.0, filled - uniform_filter(filled, int(15 * res) | 1))

    top_xy = np.asarray(cad_top.points)[:, :2]
    cad_center = top_xy.mean(axis=0)
    top_z = np.median(np.asarray(cad_top.points)[:, 2])
    keycap_height = np.percentile(q[:, 2], 90)

    seeds = {}
    for flip in (0.0, 180.0):
        for yaw in np.arange(-yaw_range, yaw_range + 1e-9, yaw_step):
            theta = np.radians(flip + yaw)
            rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            r = (top_xy - cad_center) @ rot.T
            rx_min, ry_max = r[:, 0].min(), r[:, 1].max()
            cols = ((r[:, 0] - rx_min) * res).astype(int)
            rows = ((ry_max - r[:, 1]) * res).astype(int)
            tmpl = np.zeros((rows.max() + 1, cols.max() + 1))
            tmpl[rows, cols] = 1.0
            tmpl = gaussian_filter(tmpl, blur_mm * res)
            tmpl = (tmpl > 0.1).astype(float)
            tmpl -= tmpl.mean()

            corr = fftconvolve(scan_map, tmpl[::-1, ::-1], mode="full")
            peak_row, peak_col = np.unravel_index(np.argmax(corr), corr.shape)
            score = corr[peak_row, peak_col]
            if flip in seeds and score <= seeds[flip]["global_score"]:
                continue

            # Template pixel (0, 0) sits at scan pixel (oy, ox) -> in-plane offset in the keyboard frame
            oy, ox = peak_row - tmpl.shape[0] + 1, peak_col - tmpl.shape[1] + 1
            d = np.array([qx_min - rx_min + ox / res, qy_max - ry_max - oy / res])
            cad_to_kb = np.eye(4)
            cad_to_kb[:2, :2] = rot
            cad_to_kb[:2, 3] = d - rot @ cad_center
            cad_to_kb[2, 3] = keycap_height - top_z
            kb_to_camera = np.eye(4)
            kb_to_camera[:3, :3] = axes
            kb_to_camera[:3, 3] = center
            seeds[flip] = {"source": f"template {flip:.0f}deg", "global_score": score, "yaw": yaw,
                           "transformation": np.linalg.inv(kb_to_camera @ cad_to_kb)}

    for s in seeds.values():
        print(f"Template match {s['source']}: best yaw {s['yaw']:+.1f} deg, score {s['global_score']:.0f}")
    return list(seeds.values())


# ---------------------------------------------------------------------------
# Refinement and scoring
# ---------------------------------------------------------------------------
def crop_to_keycaps(scan, top_xy_tree, transformation, margin=2.0):
    """Scan points that lie over a CAD keycap top (in CAD xy) for the given scan -> CAD pose."""
    pts = np.asarray(scan.points) @ transformation[:3, :3].T + transformation[:3, 3]
    d_xy, _ = top_xy_tree.query(pts[:, :2])
    return np.where(d_xy < margin)[0]


def multi_scale_icp(scan, cad, cad_top, seeds, voxel_sizes, max_distances):
    """Refine every global registration seed with multi-scale point-to-plane ICP.

    The case/bezel is not in the CAD, so each seed only registers the scan points over the keycaps.
    """
    device = o3d.core.Device("CUDA:0" if o3d.core.cuda.is_available() else "CPU:0")
    print(f"Refining {len(seeds)} seeds with multi-scale ICP on {device}")
    cad_t = o3d.t.geometry.PointCloud.from_legacy(cad, device=device)
    top_xy_tree = cKDTree(np.asarray(cad_top.points)[:, :2])

    estimation = o3d.t.pipelines.registration.TransformationEstimationPointToPlane()
    criteria_list = [
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.0001, 0.0001, 20),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 15),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 10),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.0000001, 0.0000001, 5),
    ]

    refined = []
    for seed in seeds:
        idx = crop_to_keycaps(scan, top_xy_tree, seed["transformation"])
        if len(idx) < 1000:
            continue
        scan_t = o3d.t.geometry.PointCloud.from_legacy(scan.select_by_index(idx), device=device)
        transform_t = o3d.core.Tensor(seed["transformation"], dtype=o3d.core.Dtype.Float32, device=device)
        result = o3d.t.pipelines.registration.multi_scale_icp(
            scan_t, cad_t,
            o3d.utility.DoubleVector(voxel_sizes), criteria_list[:len(voxel_sizes)],
            o3d.utility.DoubleVector(max_distances), transform_t, estimation)
        refined.append({
            "source": seed["source"],
            "global_score": seed["global_score"],
            "global_transformation": seed["transformation"],
            "fitness": result.fitness,
            "inlier_rmse": result.inlier_rmse,
            "transformation": result.transformation.cpu().numpy().astype(np.float64),
        })
    if not refined:
        raise RuntimeError("Registration failed: no valid global registration result to refine.")
    return refined


class KeycapReliefScorer:
    """Mean local relief of the depth image under the CAD keycap tops, in mm.

    Keycap tops sit higher than the gaps around them. With the right pose the projected CAD keycaps
    land on the locally raised parts of the scan (positive score); poses a key pitch off or flipped
    put keycaps over gaps and score lower.
    """

    def __init__(self, depth_mm, meta, cad_top, window_px=31, sample_voxel=0.7):
        valid = depth_mm > 0
        # Local mean over valid pixels only, so invalid (0) depth does not bleed into the relief
        mean = (uniform_filter(np.where(valid, depth_mm, 0.0), window_px)
                / np.maximum(uniform_filter(valid * 1.0, window_px), 1e-6))
        self.relief = np.where(valid, mean - depth_mm, 0.0)
        self.meta = meta
        self.points = np.asarray(cad_top.voxel_down_sample(sample_voxel).points)

    def __call__(self, transformation):
        camera_to_cad = np.linalg.inv(transformation)
        p = self.points @ camera_to_cad[:3, :3].T + camera_to_cad[:3, 3]
        u = self.meta["fx"] * p[:, 0] / p[:, 2] + self.meta["ppx"]
        v = self.meta["fy"] * p[:, 1] / p[:, 2] + self.meta["ppy"]
        return float(map_coordinates(self.relief, [v, u], order=1, cval=0.0).mean())


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
def pose_difference(T_a, T_b, cad_top):
    """Largest displacement (mm) of the CAD keycap-field corners between two scan -> CAD poses."""
    bb = cad_top.get_axis_aligned_bounding_box()
    lo, hi = bb.min_bound, bb.max_bound
    corners = np.array([[lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]], [lo[0], hi[1], hi[2]], [hi[0], hi[1], hi[2]]])
    a = corners @ np.linalg.inv(T_a)[:3, :3].T + np.linalg.inv(T_a)[:3, 3]
    b = corners @ np.linalg.inv(T_b)[:3, :3].T + np.linalg.inv(T_b)[:3, 3]
    return float(np.max(np.linalg.norm(a - b, axis=1)))


def distance_summary(d, thresholds):
    out = {
        "n_points": int(d.size),
        "mean_mm": float(np.mean(d)),
        "median_mm": float(np.median(d)),
        "rmse_mm": float(np.sqrt(np.mean(d ** 2))),
        "p90_mm": float(np.percentile(d, 90)),
        "p95_mm": float(np.percentile(d, 95)),
    }
    for t in thresholds:
        out[f"within_{t:g}mm_pct"] = float(100.0 * np.mean(d < t))
    return out


def compute_alignment_stats(scan, cad_mesh, cad_top, refined, threshold, thresholds=(1, 2, 3, 5),
                            footprint_tol=1.0, same_pose_mm=3.0):
    best = refined[0]
    T = best["transformation"]
    pts = np.asarray(scan.points) @ T[:3, :3].T + T[:3, 3]

    # Point -> CAD surface distance for every scan point
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(cad_mesh))
    dist = scene.compute_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()

    # Scan points that sit over a keycap top. The keyboard case/bezel is not in the CAD, so the
    # keycap region is the fair measure of fit.
    top_pts = np.asarray(cad_top.points)
    d_xy, idx = cKDTree(top_pts[:, :2]).query(pts[:, :2])
    on_keys = d_xy < footprint_tol
    height_err = pts[on_keys, 2] - top_pts[idx[on_keys], 2]

    # How much of the visible keycap surface the scan explains
    d_cov, _ = cKDTree(pts).query(top_pts)

    # Agreement between independent seeds, and the margin to the best different pose
    diffs = [pose_difference(T, r["transformation"], cad_top) for r in refined]
    agree = [r["source"] for r, d in zip(refined, diffs) if d < same_pose_mm]
    others = [(r, d) for r, d in zip(refined, diffs) if d >= same_pose_mm]
    runner_up = max(others, key=lambda x: x[0]["relief_mm"]) if others else None
    by_fitness = max(refined, key=lambda r: r["fitness"])

    camera_to_cad = np.linalg.inv(T)
    tilt = np.degrees(np.arccos(np.clip(-camera_to_cad[2, 2], -1, 1)))

    stats = {
        "selected": {
            "seed": best["source"],
            "keycap_relief_mm": best["relief_mm"],
            "icp_fitness_keycap_region": best["fitness"],
            "icp_inlier_rmse_mm": best["inlier_rmse"],
            "icp_correspondence_threshold_mm": threshold,
        },
        "runner_up_distinct_pose": None if runner_up is None else {
            "seed": runner_up[0]["source"],
            "keycap_relief_mm": runner_up[0]["relief_mm"],
            "pose_difference_mm": runner_up[1],
            "relief_ratio_best_to_runner_up": best["relief_mm"] / max(runner_up[0]["relief_mm"], 1e-6),
        },
        "welding_criterion_max_fitness": {
            "seed": by_fitness["source"],
            "fitness": by_fitness["fitness"],
            "keycap_relief_mm": by_fitness["relief_mm"],
            "pose_difference_from_selected_mm": pose_difference(T, by_fitness["transformation"], cad_top),
        },
        "scan_to_cad_all_points": distance_summary(dist, thresholds),
        "scan_to_cad_keycap_region": distance_summary(dist[on_keys], thresholds),
        "keycap_height_error_mm": {
            "mean": float(np.mean(height_err)),
            "median": float(np.median(height_err)),
            "std": float(np.std(height_err)),
        },
        "keycap_coverage_pct": {f"within_{t:g}mm": float(100.0 * np.mean(d_cov < t)) for t in thresholds},
        "keycap_region_pct_of_scan": float(100.0 * np.mean(on_keys)),
        "consistency": {
            "refined_seeds": len(refined),
            "seeds_converged_to_selected": len(agree),
            "converged_seed_sources": agree,
            "same_pose_tolerance_mm": same_pose_mm,
        },
        "pose_camera_to_cad": {
            "translation_mm": camera_to_cad[:3, 3].tolist(),
            "quaternion_xyzw": R.from_matrix(camera_to_cad[:3, :3]).as_quat().tolist(),
            "euler_xyz_deg": R.from_matrix(camera_to_cad[:3, :3]).as_euler("xyz", degrees=True).tolist(),
            "keyboard_tilt_from_optical_axis_deg": float(tilt),
        },
        "scan_to_cad_transform": T.tolist(),
    }
    return stats, dist, on_keys


def print_stats(stats, refined, cad_top):
    print("\n================ Alignment statistics ================")
    s = stats["selected"]
    print(f"Selected pose from seed '{s['seed']}'")
    print(f"  keycap relief        {s['keycap_relief_mm']:.3f} mm  (higher = CAD keycaps sit on scanned keys)")
    print(f"  ICP (keycap region)  fitness {s['icp_fitness_keycap_region']:.3f}   inlier RMSE "
          f"{s['icp_inlier_rmse_mm']:.3f} mm   (threshold {s['icp_correspondence_threshold_mm']} mm)")
    ru = stats["runner_up_distinct_pose"]
    if ru:
        print(f"  runner-up pose       relief {ru['keycap_relief_mm']:.3f} mm from '{ru['seed']}', "
              f"{ru['pose_difference_mm']:.0f} mm away -> margin x{ru['relief_ratio_best_to_runner_up']:.2f}")
    wf = stats["welding_criterion_max_fitness"]
    print(f"  max-fitness pick     '{wf['seed']}' fitness {wf['fitness']:.3f}, relief {wf['keycap_relief_mm']:.3f} mm, "
          f"{wf['pose_difference_from_selected_mm']:.1f} mm from selected")

    for key, label in [("scan_to_cad_all_points", "Scan -> CAD surface (all keyboard points, incl. case)"),
                       ("scan_to_cad_keycap_region", "Scan -> CAD surface (points over keycaps)")]:
        d = stats[key]
        print(f"\n{label}  n={d['n_points']}")
        print(f"  mean {d['mean_mm']:.2f}  median {d['median_mm']:.2f}  RMSE {d['rmse_mm']:.2f}"
              f"  p90 {d['p90_mm']:.2f}  p95 {d['p95_mm']:.2f}  [mm]")
        print("  " + "  ".join(f"<{k.split('_')[1]}: {v:.1f}%" for k, v in d.items() if k.startswith("within")))
    h = stats["keycap_height_error_mm"]
    print(f"\nKeycap height error (scan - CAD)  mean {h['mean']:+.2f}  median {h['median']:+.2f}  std {h['std']:.2f} mm")
    print("Keycap top coverage by scan       " +
          "  ".join(f"<{k.split('_')[1]}: {v:.1f}%" for k, v in stats["keycap_coverage_pct"].items()))
    print(f"Scan points over keycaps          {stats['keycap_region_pct_of_scan']:.1f}% (rest is case/bezel)")

    c = stats["consistency"]
    print(f"\nConsistency: {c['seeds_converged_to_selected']}/{c['refined_seeds']} seeds converged to the selected "
          f"pose (within {c['same_pose_tolerance_mm']} mm)")
    print("Top candidates by keycap relief:")
    best_T = refined[0]["transformation"]
    for r in refined[:6]:
        print(f"  {r['source']:16s} relief {r['relief_mm']:6.3f} mm   ICP fitness {r['fitness']:.3f}  "
              f"RMSE {r['inlier_rmse']:.3f} mm   {pose_difference(best_T, r['transformation'], cad_top):6.1f} mm from best")

    p = stats["pose_camera_to_cad"]
    print("\nCAD pose in camera optical frame")
    print(f"  translation (mm)      {np.round(p['translation_mm'], 2)}")
    print(f"  quaternion (x,y,z,w)  {np.round(p['quaternion_xyzw'], 4)}")
    print(f"  euler xyz (deg)       {np.round(p['euler_xyz_deg'], 2)}")
    print(f"  keyboard tilt from optical axis {p['keyboard_tilt_from_optical_axis_deg']:.1f} deg")
    print("======================================================\n")


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
def error_colors(dist, on_keys, max_err=5.0):
    """Blue (0 mm) -> green -> red (>= max_err). Points off the keycaps are grey."""
    x = np.clip(dist / max_err, 0, 1)[:, None]
    blue, green, red = np.array([0, 0.3, 1]), np.array([0, 0.9, 0.2]), np.array([1, 0, 0])
    colors = np.where(x < 0.5, blue + (green - blue) * x * 2, green + (red - green) * (x - 0.5) * 2)
    colors[~on_keys] = [0.6, 0.6, 0.6]
    return colors


def keycap_overlay_image(depth_mm, meta, cad_top, transformation):
    """Depth relief image with the CAD keycap tops projected on it (blue)."""
    relief = depth_mm - uniform_filter(depth_mm, 25)
    grey = np.clip(128 - relief * 40, 0, 255).astype(np.uint8)
    img = np.stack([grey] * 3, axis=-1)
    camera_to_cad = np.linalg.inv(transformation)
    p = np.asarray(cad_top.voxel_down_sample(0.7).points)
    p = p @ camera_to_cad[:3, :3].T + camera_to_cad[:3, 3]
    u = (meta["fx"] * p[:, 0] / p[:, 2] + meta["ppx"]).astype(int)
    v = (meta["fy"] * p[:, 1] / p[:, 2] + meta["ppy"]).astype(int)
    ok = (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0])
    img[v[ok], u[ok]] = (0.45 * img[v[ok], u[ok]] + 0.55 * np.array([0, 160, 255])).astype(np.uint8)
    return img


def build_views(scene, keyboard, cad_mesh, refined, dist, on_keys):
    """Geometries in the camera optical frame so views match the depth image."""
    camera_to_cad = np.linalg.inv(refined[0]["transformation"])
    camera_to_cad_seed = np.linalg.inv(refined[0]["global_transformation"])

    scene_grey = copy.deepcopy(scene)
    scene_grey.paint_uniform_color([0.65, 0.65, 0.65])
    kb = copy.deepcopy(keyboard)
    kb.paint_uniform_color(SCAN_COLOR)

    def cad_at(T):
        m = copy.deepcopy(cad_mesh)
        m.transform(T)
        m.compute_vertex_normals()
        m.paint_uniform_color(CAD_COLOR)
        return m

    err = copy.deepcopy(keyboard)
    err.colors = o3d.utility.Vector3dVector(error_colors(dist, on_keys))
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50.0)
    frame.transform(camera_to_cad)

    return [
        ("1 Segmentation: keyboard (orange) vs removed scene", [scene_grey, kb]),
        (f"2 Global seed ({refined[0]['source']}): scan (orange), CAD (blue)", [kb, cad_at(camera_to_cad_seed)]),
        ("3 Final ICP alignment: scan (orange), CAD (blue)", [kb, cad_at(camera_to_cad), frame]),
        ("4 Error map over keycaps: blue 0 mm -> red 5+ mm, grey = case", [err, frame]),
    ]


def camera_view(center):
    # Look down the optical axis from the camera, image "up" is -y
    return dict(front=[0.0, 0.0, -1.0], lookat=center, up=[0.0, -1.0, 0.0], zoom=0.55)


def show_views(views, center, overlay):
    for title, geoms in views:
        print(f"Showing: {title}  (close the window to continue)")
        o3d.visualization.draw_geometries(geoms, window_name=title, width=1280, height=800,
                                          **camera_view(center))
    print("Showing: 5 CAD keycaps (blue) projected on the depth relief image")
    o3d.visualization.draw_geometries([o3d.geometry.Image(overlay)], window_name="5 CAD keycaps on depth image",
                                      width=overlay.shape[1], height=overlay.shape[0])


def save_views(views, center, overlay, save_dir, width=1280, height=800):
    import imageio.v2 as imageio
    from open3d.visualization import rendering
    imageio.imwrite(os.path.join(save_dir, "5_keycaps_on_depth.png"), overlay)

    point_mat = rendering.MaterialRecord()
    point_mat.shader = "defaultUnlit"
    point_mat.point_size = 2.0
    mesh_mat = rendering.MaterialRecord()
    mesh_mat.shader = "defaultLit"

    eyes = {"top": np.array([0.0, 0.0, -1.0]), "oblique": np.array([0.35, 0.6, -0.72])}
    names = ["1_segmentation", "2_global_seed", "3_final_alignment", "4_error_map"]
    renderer = rendering.OffscreenRenderer(width, height)
    renderer.scene.set_background([1, 1, 1, 1])
    for name, (title, geoms) in zip(names, views):
        renderer.scene.clear_geometry()
        for i, g in enumerate(geoms):
            mat = point_mat if isinstance(g, o3d.geometry.PointCloud) else mesh_mat
            renderer.scene.add_geometry(f"g{i}", g, mat)
        for tag, front in eyes.items():
            eye = center + front / np.linalg.norm(front) * 420.0
            renderer.setup_camera(60.0, center, eye, [0.0, -1.0, 0.0])
            imageio.imwrite(os.path.join(save_dir, f"{name}_{tag}.png"), np.asarray(renderer.render_to_image()))


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--depth", default=os.path.join(DATA_DIR, "keyboard_d405_Depth.raw"))
    parser.add_argument("--meta", default=os.path.join(DATA_DIR, "keyboard_d405_Depth_metadata.csv"))
    parser.add_argument("--cad", default=os.path.join(DATA_DIR, "Tenkeyless_KB_full.stl"))
    parser.add_argument("--depth-unit-mm", type=float, default=0.1, help="D405 default depth unit is 0.1 mm")
    parser.add_argument("--max-depth-mm", type=float, default=400.0)
    parser.add_argument("--voxel", type=float, default=5.0, help="RANSAC voxel size [mm]")
    parser.add_argument("--attempts", type=int, default=48, help="parallel RANSAC attempts")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--cad-points", type=int, default=300000)
    parser.add_argument("--no-template", action="store_true",
                        help="RANSAC seeds only, as in the welding pipeline (for comparison)")
    parser.add_argument("--no-vis", action="store_true", help="skip interactive windows")
    parser.add_argument("--save-dir", default=None, help="save renders, stats.json and segmented scan here")
    args = parser.parse_args()

    # ICP scales [mm]: coarse -> fine
    icp_voxel_sizes = [4.0, 2.0, 1.0, 0.5]
    icp_max_distances = [20.0, 8.0, 4.0, 2.0]

    # --- Load and segment the capture ---------------------------------------------
    scene, depth_mm, meta = load_d405_depth_as_point_cloud(args.depth, args.meta, args.depth_unit_mm, args.max_depth_mm)
    print(f"Capture has {len(scene.points)} points.")
    keyboard, scene_down = segment_keyboard(scene)
    obb = keyboard.get_minimal_oriented_bounding_box()
    print(f"Keyboard cluster: {len(keyboard.points)} points, extent {np.round(np.sort(obb.extent)[::-1], 1)} mm")

    # --- Load CAD -----------------------------------------------------------------
    cad_mesh, cad_target, cad_top = load_visible_cad(args.cad, args.cad_points)
    print(f"CAD extent {np.round(cad_mesh.get_axis_aligned_bounding_box().get_extent(), 1)} mm, "
          f"target {len(cad_target.points)} points")

    # --- Global registration seeds ------------------------------------------------------
    seeds = global_registration(keyboard, cad_target, args.voxel, args.attempts, args.workers)
    if not args.no_template:
        seeds += template_match_registration(keyboard, cad_top)

    # --- Refine and rank ---------------------------------------------------------------
    refined = multi_scale_icp(keyboard, cad_target, cad_top, seeds, icp_voxel_sizes, icp_max_distances)
    scorer = KeycapReliefScorer(depth_mm, meta, cad_top)
    for r in refined:
        r["relief_mm"] = scorer(r["transformation"])
    refined.sort(key=lambda r: r["relief_mm"], reverse=True)

    # --- Statistics -----------------------------------------------------------------
    stats, dist, on_keys = compute_alignment_stats(keyboard, cad_mesh, cad_top, refined, icp_max_distances[-1])
    print_stats(stats, refined, cad_top)

    # --- Visualize ------------------------------------------------------------------
    views = build_views(scene_down, keyboard, cad_mesh, refined, dist, on_keys)
    overlay = keycap_overlay_image(depth_mm, meta, cad_top, refined[0]["transformation"])
    center = keyboard.get_center()
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        with open(os.path.join(args.save_dir, "stats.json"), "w") as f:
            json.dump(stats, f, indent=2)
        o3d.io.write_point_cloud(os.path.join(args.save_dir, "keyboard_segmented.ply"), keyboard)
        save_views(views, center, overlay, args.save_dir)
        print(f"Saved results to {args.save_dir}")
    if not args.no_vis:
        show_views(views, center, overlay)


if __name__ == "__main__":
    main()
