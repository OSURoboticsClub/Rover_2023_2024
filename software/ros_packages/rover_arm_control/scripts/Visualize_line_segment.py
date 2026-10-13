"""
Visulize line segments
OMI Project: Towards Intelligent Robotic Welding - Weld Seam Tracking
Authors: Jared Northrop
Year: 2526

This set of functions allows a weld path defined in CAM to be visualized with the point cloud captures of 
the object and find the closest points along the weld path. 
"""
import numpy as np
import open3d as o3d
import Test_open3d_v3 as test_o3d
import copy


def read_segments_from_file(path):
    """Read in set a of line segments from CAM output file.

    Parameters
    ----------
    path : str
        The path to the file containing the CAM output. 

    Returns
    -------
    segments : list of tuples
        A list of line segments, where each segment is represented as a tuple of the two endpoints. 
    """
    segments = []

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # split on semicolons
            parts = [p.strip() for p in line.split(";") if p.strip()]
            if len(parts) < 2:
                continue

            p1 = np.fromstring(parts[0], sep=",")
            p2 = np.fromstring(parts[1], sep=",")

            if p1.size != 3 or p2.size != 3:
                raise ValueError(f"Bad line format:\n{line}")

            segments.append((p1.tolist(), p2.tolist()))

    return segments

def lineset_from_segments(segments, color=[0, 1, 0]):
    points = []
    lines = []
    point_index = {}

    def idx(p):
        key = tuple(np.round(p, 6))
        if key not in point_index:
            point_index[key] = len(points)
            points.append(p)
        return point_index[key]

    for p1, p2 in segments:
        i1 = idx(p1)
        i2 = idx(p2)
        lines.append([i1, i2])

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color] * len(lines))

    return ls

def point_to_line_segment(pcd, p1, p2):
    #vector from p1 to p2
    ab = p2 - p1
    #get the magnitude squared
    ab_len2 = np.dot(ab, ab)

    if ab_len2 == 0:
        return np.linalg.norm(pcd - p1, axis=1)

    t = np.dot(pcd - p1, ab) / ab_len2
    t = np.clip(t, 0.0, 1.0)

    closest = p1 + np.outer(t, ab)
    return np.linalg.norm(pcd - closest, axis=1)

def closest_indices_along_segments(pcd, segments, spacing=0.01):
    """
    pcd       : open3d.geometry.PointCloud
    segments  : list of (a, b) tuples
    spacing   : meters (0.01 = 1 cm)

    Returns:
        unique_indices (list)
    """

    points = np.asarray(pcd.points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    collected_indices = []

    for a, b in segments:
        a = np.array(a)
        b = np.array(b)

        direction = b - a
        length = np.linalg.norm(direction)
        if length == 0:
            continue
        direction /= length

        sample_distances = np.arange(0, length, spacing)
        samples = a + np.outer(sample_distances, direction)

        for sample in samples:
            _, idx, _ = kdtree.search_knn_vector_3d(sample, 1)
            collected_indices.append(idx[0])

    # Remove duplicates
    # unique_indices = list(set(collected_indices))
    unique_indices = list(dict.fromkeys(collected_indices))


    return unique_indices

def indices_to_lineset(pcd, idxs, color=[1, 0, 0]):
    points = np.asarray(pcd.points)[idxs]

    lines = [[i, i + 1] for i in range(len(points) - 1)]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    line_colors = np.tile(color, (len(lines), 1))
    line_set.colors = o3d.utility.Vector3dVector(line_colors)

    return line_set

def format_point(p):
    return f"{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}"

def export_segments_to_file(lineset, path, angle = 0.0):
    with open(path, "w") as f:
        lines = np.asarray(lineset.lines)
        points = np.asarray(lineset.points)
        for i, j in lines:
            p1 = format_point(points[i])
            p2 = format_point(points[j])
            #Need to pick orientation of each point along the line.
            # This needs to be the same for all points between the same vertexes as provided by cam. 
            # Either need to process points differently to know which vertex group they belong to or a function to determine this. 
            # Orientation should be perpendicular to the line and facing inward towards the origin/center.
            # The above defines only a single rotation, (roll is arbitrary and should be the same for all points) 
            # Angle out of the plane of the weld seam should be provided by the user.  
            f.write(f"{p1}; {p2}; 0, 0, 1;\n")


if __name__ == "__main__":
    segments = read_segments_from_file("/home/jn2/college/Cobot_Welding_Masters_Project/data/1001.dat")
    for p1, p2 in segments:
        print(f"Segment from {p1} to {p2}")

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=200.0,   # scale to your scene
    origin=[0, 0, 0]
    )

    line_set = lineset_from_segments(segments)
    # o3d.visualization.draw_geometries([line_set, origin])
    #load target object from mesh library and prepare dataset
    voxel_size = 50.0
    object_path = "/home/jn2/college/Cobot_Welding_Masters_Project/meshes/Shortened-25038-WA-44-1.stl"
    object_mesh = test_o3d.load_model_as_point_cloud(object_path, n_points=1000000)
    print(f"object has {len(object_mesh.points)} points.")
    object_down, object_fpfh, org_object = test_o3d.preprocess_point_cloud(object_mesh, voxel_size)
    print(f"object downsampled to {len(object_down.points)} points.")
    # draw_registration_result(object_down, pc_downs[0], np.identity(4))

    # o3d.visualization.draw_geometries([org_object, origin])

    # print("Object center:", object_down.get_center())
    # print("Object bounds:", object_down.get_min_bound(), object_down.get_max_bound())

    transform = np.array([
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    # test_o3d.draw_registration_result(line_set, org_object, transform)

    clone_object = copy.deepcopy(org_object)

    clone_object.transform(transform)
    # test_o3d.draw_registration_result(line_set, clone_object, np.eye(4))

    #Find Close Points
    # print(clone_object.points)

    threshold = 2
    points = np.asarray(clone_object.points)
    mask = np.zeros(len(points), dtype=bool)

    for a, b in segments:
        d = point_to_line_segment(points, np.array(a), np.array(b))
        mask |= d < threshold

    near_line_points = points[mask]
    idxs = np.where(mask)[0]
    near_pcd = clone_object.select_by_index(idxs)
    removed_pcd = clone_object.select_by_index(idxs, invert=True)

    # near_pcd = o3d.geometry.PointCloud()
    # near_pcd.points = o3d.utility.Vector3dVector(near_line_points)
    print(f"Found {len(near_pcd.points)} points near the line segments.")

    o3d.visualization.draw_geometries([near_pcd])
    test_o3d.draw_registration_result(near_pcd, removed_pcd, np.eye(4))

    idx = closest_indices_along_segments(clone_object, segments[1:-1], spacing=10)
    near_lin_pcd = clone_object.select_by_index(idx)
    removed_lin_pcd = clone_object.select_by_index(idx, invert=True)
    print(f"Found closest {len(near_lin_pcd.points)} points on line segment")

    o3d.visualization.draw_geometries([near_lin_pcd])
    test_o3d.draw_registration_result(near_lin_pcd, removed_lin_pcd, np.eye(4))

    near_line_set = indices_to_lineset(clone_object, idx, [1, 0.706, 0])
    o3d.visualization.draw_geometries([
        clone_object,
        near_line_set
    ])

    export_segments_to_file(near_line_set, "/home/jn2/college/Cobot_Welding_Masters_Project/data/1001_near.dat")



