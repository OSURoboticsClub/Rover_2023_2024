"""
Open3D  Point Cloud Processing and Registration
OMI Project: Towards Intelligent Robotic Welding - Weld Seam Tracking
Authors: Jared Northrop
Year: 2526

This set of functions allows point cloud captures to be registred to the CAD model for weld seam tracking and
alignment of the coordinate frames. 
"""
import open3d as o3d
import numpy as np
import time
import os
import copy
try:
    import zivid  # only needed for Zivid .zdf captures
except ImportError:
    zivid = None
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor
import imageio.v2 as imageio

CAMERA_TYPE = "realsense"  # or "azure", "kinect", "openni"
PC_SEGMENTED = None
PC_DOWNS = None
ORG_PC_SEGMENTED = None
PC_FPFHS = None
OBJECT_DOWN = None
ORG_OBJECT = None
OBJECT_FPFH = None

def get_pc_from_zdf(path : str, visualize=False) -> o3d.geometry.PointCloud:
    app = zivid.Application()
    frame = zivid.Frame(path)
    point_cloud = frame.point_cloud()

    pc_unstructured = point_cloud.to_unorganized_point_cloud()
    # Extract already-flat data
    xyz = pc_unstructured.copy_data("xyz")
    rgba = pc_unstructured.copy_data("rgba")

    # Zivid already flattened it → no reshape needed
    points = xyz
    colors = rgba[:, :3] / 255.0

    # Convert to Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize
    if visualize:
        o3d.visualization.draw_geometries([pcd])
    return pcd


def zdf_to_open3d_cloud(zdf_path: str) -> o3d.geometry.PointCloud:
    app = zivid.Application()

    # Load the ZDF file
    frame = zivid.Frame(zdf_path)
    pc = frame.point_cloud()

    # Get organized data
    xyz = pc.copy_data("xyz")
    rgba = pc.copy_data("rgba")

    # Flatten to Open3D format
    points = xyz.reshape(-1, 3)
    colors = rgba.reshape(-1, 4)[:, :3] / 255.0

    # Remove invalid points (important)
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]
    colors = colors[mask]

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])

    return pcd

def capture_rgbd_frame():
    """
    Capture a single RGB-D frame using Open3D's sensor API.
    """
    if CAMERA_TYPE == "realsense":
        rs = o3d.t.io.RealSenseSensor.list_devices()
        if not rs:
            raise RuntimeError("No RealSense device found!")

        sensor = o3d.t.io.RealSenseSensor()
        sensor.start_capture(True)
        time.sleep(2.0)

        frame = sensor.capture_frame(True, True)
        sensor.stop_capture()

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            frame.color.to_legacy(),
            frame.depth.to_legacy(),
            convert_rgb_to_intensity=False
        )
        return rgbd

    else:
        raise NotImplementedError(f"Camera type '{CAMERA_TYPE}' not supported.")
    
def rgbd_to_pointcloud(rgbd, intrinsic=None):
    if intrinsic is None:
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
        )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])
    return pcd


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    print(":: Estimate Original normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    # print(":: Compute Original FPFH feature with search radius %.3f." % radius_feature)
    # org_pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    #     pcd,
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh, pcd

def segment_object_from_scene(pcd, visualize=True):

    print("Segmenting object from scene...")

    # Remove distant background by depth
    # pts = np.asarray(pcd.points)
    # z = pts[:, 2]
    # depth_mask = z < 3.0   # keep points closer than 1 meter
    # pcd = pcd.select_by_index(np.where(depth_mask)[0])

    # Remove dominant plane (table)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.1,
        ransac_n=3,
        num_iterations=2000
    )
    table = pcd.select_by_index(inliers)
    rest = pcd.select_by_index(inliers, invert=True)

    if visualize:
        table.paint_uniform_color([1, 0, 0])  # red
        rest.paint_uniform_color([0, 0, 1])   # blue
        print("  - Showing plane vs everything not on plane...")
        o3d.visualization.draw_geometries([table, rest])

    # Remove statistical noise (helps segmentation)
    rest, _ = rest.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)

    if visualize:
        rest.paint_uniform_color([0, 0, 1])   # blue
        print("  - Showing remaining points after noise removal...")
        o3d.visualization.draw_geometries([rest])

    # remove far-away leftover background
    pts = np.asarray(rest.points)
    center = pts.mean(axis=0)

    radius = 0.1   # meters from center (tunable!)
    dist = np.linalg.norm(pts - center, axis=1)
    dist = 0.2
    rest = rest.select_by_index(np.where(pts < dist)[0])

    # Cluster remaining foreground points
    labels = np.array(rest.cluster_dbscan(eps=0.02, min_points=40))

    if labels.max() < 0:
        raise RuntimeError("No clusters found after background removal!")

    largest_cluster_id = np.bincount(labels[labels >= 0]).argmax()
    object_cluster = rest.select_by_index(
        np.where(labels == largest_cluster_id)[0]
    )

    if visualize:
        object_cluster.paint_uniform_color([0, 1, 0])  # green
        print("  - Showing isolated object cluster...")
        o3d.visualization.draw_geometries([object_cluster])

    return object_cluster

def segment_welding_object(pcd, visualize=True,  eps = 2.0, min_points = 100, dist_threshold = 0.1, ransac_n = 3, 
        num_iterations = 2000, nb_neighbors = 30, std_ratio = 1.5, remove_plane = True, cluster = True):
    print("Sementing Point Cloud for Welding Object")
    if visualize:
        print_progress = True
    else:
        print_progress = False
    # Remove dominant plane (table)
    if remove_plane:
        print(" - Removing Dominant Plane")
        
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=dist_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )
        table = pcd.select_by_index(inliers)
        rest = pcd.select_by_index(inliers, invert=True)

        if visualize:
            table.paint_uniform_color([1, 0, 0])  # red
            rest.paint_uniform_color([0, 0, 1])   # blue
            print("  - Showing plane vs everything not on plane...")
            o3d.visualization.draw_geometries([table, rest])
    else:
        rest = pcd

    # Remove statistical noise (helps segmentation)
    print(" - Removing statistical noise")
    rest, _ = rest.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio, print_progress=print_progress)
    if visualize:
        rest.paint_uniform_color([0, 0, 1])   # blue
        print("  - Showing remaining points after noise removal...")
        o3d.visualization.draw_geometries([rest])
        
    # Cluster remaining foreground points
    if cluster:
        print(" - Clustering remaining points")
        labels = np.array(rest.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress))
        num_clusters = np.unique(labels[labels >= 0]).size
        print(f"  - Found {num_clusters} clusters.")

        if labels.max() < 0:
            raise RuntimeError("No clusters found after background removal!")

        clusters = []
        for cluster_id in range(labels.max() + 1):
            cluster = rest.select_by_index(
                np.where(labels == cluster_id)[0]
            )
            clusters.append(cluster)
            if visualize:
                print(f"  - Showing cluster {cluster_id}...")
                cluster.paint_uniform_color([1,0,0])  # red color
                the_rest = rest.select_by_index(np.where(labels != cluster_id)[0])
                the_rest.paint_uniform_color([0, 0, 1])   # blue
                o3d.visualization.draw_geometries([cluster, the_rest])
        return clusters
    else:
        return [rest]

def load_model_as_point_cloud(path, n_points=5000, visualize = False):
    if path.lower().endswith(".stl"):
        mesh = o3d.io.read_triangle_mesh(path)
        mesh.compute_vertex_normals()
        mesh.scale(1.0, center=mesh.get_center())
        # Sample points uniformly from the mesh
        pcd = mesh.sample_points_uniformly(n_points) # use_triangle_normal=True
        pcd.paint_uniform_color([0, 1, 0])  # green

        # Visualize
        if visualize:
            o3d.visualization.draw_geometries([pcd])
        return pcd
    else:
        # Fallback for ply/pcd
        return o3d.io.read_point_cloud(path)


def prepare_dataset(voxel_size, library_dir="meshes"):
    print(":: Load two point clouds and disturb initial pose.")
    model = None
    for fname in os.listdir(library_dir):
        if not fname.lower().endswith((".ply", ".pcd", "stl")):
            continue
        print(f"Comparing to {fname}...")
        # path = os.path.join(library_dir, fname)
        # model = o3d.io.read_point_cloud(path)
        # model_down, model_fpfh = preprocess_point_cloud(model, voxel_size)
        path = os.path.join(library_dir, fname)
        model = load_model_as_point_cloud(path, 100000)
        # Color by normal for debugging
        # normals = model_down.normals
        # colors = [(abs(n[0]), abs(n[1]), abs(n[2])) for n in normals]
        # model_down.colors = o3d.utility.Vector3dVector(colors)
        # o3d.visualization.draw_geometries([model_down])

    # demo_icp_pcds = o3d.data.DemoICPPointClouds()

    # source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    target = model
    source_frame = capture_rgbd_frame()
    source_model = rgbd_to_pointcloud(source_frame)
    # source = segment_object_from_scene(source_model, visualize=True)
    source = source_model
    # source = model
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh, org_source = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh, org_target = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh, org_source, org_target

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size, with_scaling=True):
    distance_threshold = voxel_size * 1.5
    # print("RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, False,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.90),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold),
            # o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.79)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_transform=np.identity(4)):
    distance_threshold = voxel_size * 0.4
    print("Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result
def run_parallel_registration(j, clusters, source_down, source_fpfh, target_down, target_fpfh, voxel_size, with_scaling=True):
    print(f"[Worker {j}] Starting")

    for i, cluster in enumerate(clusters):
        result = execute_global_registration(
            source_down[i], target_down,
            source_fpfh[i], target_fpfh,
            voxel_size, with_scaling
        )
        # print(f"[Worker {j}] Global Registration for cluster {i}: fitness={fast_ransac.fitness}, inlier_rmse={fast_ransac.inlier_rmse}")
    return result 

def make_transform_icp_callback(transform_list):
    def callback(loss_log_map):
        Trans = loss_log_map["transformation"]
        transform_list.append(Trans.cpu().numpy().copy())
        # print(f"Transform: {transform_list}")
        return False
    return callback


def setup_visualizer(width = 1920, height=1080):
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        width=width,
        height=height,
        visible = False
    )
    return vis

def set_fixed_view(vis):
    ctr = vis.get_view_control()
    ctr.set_zoom(0.25) #zoom (small is in)
    ctr.set_front([0.0, 0.0, -1.0]) #Vector for camera direction
    ctr.set_lookat([90.06, 0.1903, 519.94]) #point to position camera from 
    ctr.set_up([0.0, 1.0, 0.0]) #up direction vector

def print_camera_params(vis):
    ctr = vis.get_view_control()

    print("Front:", ctr.get_front())
    print("Lookat:", ctr.get_lookat())
    print("Up:", ctr.get_up())
    print("Zoom:", ctr.get_zoom())

def print_camera(vis):
    ctr = vis.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()

    print("Intrinsic matrix:\n", params.intrinsic.intrinsic_matrix)
    print("Extrinsic matrix:\n", params.extrinsic)


def setup_vis_with_fixed_target(target, source, width=1920, height=1080):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height, visible=False)

    vis.add_geometry(target)
    vis.add_geometry(source)

    vis.reset_view_point(False)

    set_fixed_view(vis)

    vis.poll_events()
    vis.update_renderer()

    return vis

def render_frame_option_b_reset(
    vis,
    source,
    base_source,
    T,
    filename
):
    # Reset source to original pose
    source.points = o3d.utility.Vector3dVector(
        np.asarray(base_source.points)
    )

    # Apply absolute transform
    source.transform(T)

    # Notify Open3D
    vis.update_geometry(source)

    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(filename, do_render=True)
    # print_camera(vis)

def render_frame(vis, source, target, transformation, filename):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)

    vis.clear_geometries()
    vis.add_geometry(source_temp)
    vis.add_geometry(target_temp)


    vis.poll_events()
    vis.update_renderer()

    vis.capture_screen_image(filename, do_render=True)

if __name__ == "__main__":

    voxel_size = 0.008 # means 5cm for this dataset (possibly for realsense or cad)
    #** Initial Welding test **
    """
    Welding data is in mm scale so all parameters need to be in mm not m. 
    """

    zivid_file = "/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/base_plate_far_test.zdf"

    #load target point cloud from zdf
    pc_zivid = get_pc_from_zdf(zivid_file)
    pc_segmented = [pc_zivid]
    # pc_segmented = segment_welding_object(pc_zivid, visualize=True, eps = 10.0, min_points = 500, dist_threshold = 2.0, remove_plane=False, cluster = True)

    #load a single mesh-segmented point cloud from ply (for testing)
    # pc_segmented = []
    # pc_segmented.append(o3d.io.read_point_cloud("/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/mid_orbit_3_segment_0.ply"))

    pc_downs = []
    pc_fpfhs = []
    org_pc_segmented = []
    voxel_size = 50.0
    for i, cluster in enumerate(pc_segmented):
        # o3d.io.write_point_cloud(f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/mid_orbit_3_segment_{i}.ply", cluster)
        print(f"Cluster {i} has {len(cluster.points)} points.")
        pc_down, pc_fpfh, org_pc = preprocess_point_cloud(cluster, voxel_size)
        pc_downs.append(pc_down)
        pc_fpfhs.append(pc_fpfh)
        org_pc_segmented.append(org_pc)
        print(f"Cluster {i} downsampled to {len(pc_downs[i].points)} points.")

    #load target object from mesh library and prepare dataset
    voxel_size = 50.0
    object_path = "/home/jn2/college/Cobot_Welding_Masters_Project/meshes/Shortened-25038-WA-44-1.stl"
    object_mesh = load_model_as_point_cloud(object_path, n_points=10000000)
    print(f"object has {len(object_mesh.points)} points.")
    object_down, object_fpfh, org_object = preprocess_point_cloud(object_mesh, voxel_size)
    print(f"object downsampled to {len(object_down.points)} points.")
    # draw_registration_result(object_down, pc_downs[0], np.identity(4))

    best_transformation = None
    best_fitness = -0.01
    #Not Parallelized version for testing
    # for j in range(3):
    #     for i, cluster in enumerate(pc_segmented):
    #         print(f"Attempting registration with cluster {i}...")
    #         #print("Source points:", len(np.asarray(object_down.points)))
    #         #print("Target points:", len(np.asarray(pc_downs[i].points)))
    #         voxel_size = 9.0
    #         try:
    #             fast_ransac = execute_fast_global_registration(pc_downs[i], object_down,
    #                                                         pc_fpfhs[i], object_fpfh,
    #                                                         voxel_size)
    #         except Exception as e:
    #             print(f"Error during registration with cluster {i}: {e}")
    #             continue
    #         print(f"Global Registration for cluster {i}: {fast_ransac}")
    #         #print(f"RANSAC transformation for cluster {i}: {fast_ransac.transformation}")
    #         # draw_registration_result(pc_downs[i], object_down, fast_ransac.transformation)
    #         num_iter = 10
    #         result_transform = fast_ransac.transformation 
    #         for k in range(num_iter):
    #             voxel_size = num_iter - k  # Decrease voxel size each iteration
    #             try:
    #                 refined_result = refine_registration(org_pc_segmented[i], org_object,
    #                                                     pc_fpfhs[i], object_fpfh,
    #                                                     voxel_size, result_transform)
    #             except Exception as e:
    #                 print(f"Error during refinement with cluster {i}: {e}")
    #                 continue
    #             result_transform = refined_result.transformation
    #             if refined_result.fitness > best_fitness:
    #                 best_fitness = refined_result.fitness
    #                 best_transformation = refined_result.transformation
    #             print(f"Refinement iteration {k} with voxel size = {voxel_size}")
    #             #print(f"Refined transformation for cluster {i}: {refined_result.transformation}")
    #             # draw_registration_result(pc_downs[i], object_down, refined_result.transformation)

    #define global variables for multiprocessing
    PC_SEGMENTED = pc_segmented
    PC_DOWNS = pc_downs
    ORG_PC_SEGMENTED = org_pc_segmented
    PC_FPFHS = pc_fpfhs
    OBJECT_DOWN = object_down
    ORG_OBJECT = org_object
    OBJECT_FPFH = object_fpfh

    print("Starting parallel registration with multiprocessing")
    best_global_transformation = None
    best_global_fitness = -0.01
    with ThreadPoolExecutor(max_workers=12) as ex:
        results = list(ex.map(run_parallel_registration, range(48), [PC_SEGMENTED]*48, [PC_DOWNS]*48, [PC_FPFHS]*48, [OBJECT_DOWN]*48, [OBJECT_FPFH]*48, [voxel_size]*48))
    
    for i, res in enumerate(results):
        print(f"Result from worker {i}: fitness={res.fitness}, inlier_rmse={res.inlier_rmse}")
        if res.fitness > best_global_fitness and abs(1-res.fitness) > 0.000000001:
            best_global_fitness = res.fitness
            best_global_transformation = res.transformation
    
    print(f"Best fitness from parallel registration: {best_global_fitness}")
    # print(f"Best transformation from parallel registration: {best_global_transformation}")

    print("ICP with Tensor")
    # Example callback_after_iteration lambda function:
    device = o3d.core.Device("CUDA:0" if o3d.core.cuda.is_available() else "CPU:0")
    org_pcd_t = o3d.t.geometry.PointCloud.from_legacy(org_pc_segmented[0], device=device)
    org_object_t = o3d.t.geometry.PointCloud.from_legacy(org_object, device=device)

    callback_after_iteration = lambda updated_result_dict : print("Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
    updated_result_dict["iteration_index"].item(),
    updated_result_dict["fitness"].item(),
    updated_result_dict["inlier_rmse"].item()
    ))
    callback_after_iteration_multi = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
    loss_log_map["iteration_index"].item(),
    loss_log_map["scale_index"].item(),
    loss_log_map["scale_iteration_index"].item(),
    loss_log_map["fitness"].item(),
    loss_log_map["inlier_rmse"].item()
    ))
    #create ICP parameters
    estimation = o3d.t.pipelines.registration.TransformationEstimationPointToPlane()
    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(relative_fitness = 1e-6,relative_rmse = 1e-6, max_iteration=50)
    criteria_list = [
    o3d.t.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=20),
    o3d.t.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    o3d.t.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 10),
    o3d.t.pipelines.registration.ICPConvergenceCriteria(0.0000001, 0.0000001, 5)
    ]
    max_correspondence_distance = 3.0
    max_correspondence_distances = o3d.utility.DoubleVector([50, 20, 7.5, 1.5])
    voxel_size_t = o3d.utility.DoubleVector([30, 10, 5, 1])

    best_fitness = -1.0
    best_transformation = None
    best_fitness_multi = -1.0
    best_transformation_multi = None
    for i, res in enumerate(results):
        if abs(res.fitness-1.0) < 0.000000001 or abs(res.fitness) < 0.000000001:
            continue
        transform_t = o3d.core.Tensor(
            res.transformation, 
            dtype=o3d.core.Dtype.Float32, 
            device=device
        )
        #Vanilla ICP
        # result = o3d.t.pipelines.registration.icp(
        #     org_pcd_t, 
        #     org_object_t, 
        #     max_correspondence_distance, 
        #     res.transformation, 
        #     estimation, 
        #     criteria, 
        #     voxel_size,
        #     callback_after_iteration
        # )
        # print("ICP Result: Fitness: {}, Inlier RMSE: {}, Transformation:\n{}".format(
        #     result.fitness,
        #     result.inlier_rmse,
        #     result.transformation.cpu().numpy()
        # ))
        # if result.fitness > best_fitness:
        #     best_fitness = result.fitness
        #     best_transformation = result.transformation.cpu().numpy()
        #Multi-Scale ICP
        # draw_registration_result(org_pc_segmented[0], org_object, res.transformation)
        result_multi = o3d.t.pipelines.registration.multi_scale_icp(
            org_pcd_t, 
            org_object_t,
            voxel_size_t,
            criteria_list,
            max_correspondence_distances,
            transform_t,
            estimation,
            # callback_after_iteration_multi
        )
        print("Multi-Scale ICP Result: Iteration: {}, Fitness: {}, Inlier RMSE: {}".format(
            i,
            result_multi.fitness,
            result_multi.inlier_rmse,
        ))
        if result_multi.fitness > best_fitness_multi:
            best_fitness_multi = result_multi.fitness
            best_transformation_multi = result_multi.transformation.cpu().numpy()

    # print(f"Best fitness: {best_fitness}")
    print(f"Best fitness multi-scale: {best_fitness_multi}")
    # print(f"Best transformation: {best_transformation}")
    # print(f"Best transformation multi-scale: {best_transformation_multi}")
    
    # draw_registration_result(pc_downs[0], object_down, best_transformation)
    # draw_registration_result(org_pc_segmented[0], org_object, best_transformation)
    draw_registration_result(org_pc_segmented[0], org_object, best_transformation_multi)

    
    #Create Gif of Transforms
    # vis = setup_visualizer()
    temp_target = copy.deepcopy(org_object)
    temp_target.paint_uniform_color([0, 0.651, 0.929])  
    temp_source = copy.deepcopy(org_pc_segmented[0])
    temp_source.paint_uniform_color([1, 0.706, 0])
    # vis.add_geometry(temp_target)
    # vis.add_geometry(temp_source)
    # set_fixed_view(vis)
    # vis.reset_view_point(False)
    vis = setup_vis_with_fixed_target(temp_source, temp_target)


    file_location = "/home/jn2/college/Cobot_Welding_Masters_Project/Captures/post_processing_gif/"
    num_captures = 0
    for i, res in enumerate(results):
        if res.fitness < 0.01 or abs(res.fitness-1.0) < 0.000000001:
            continue
        temp_inv_transform = res.transformation
        temp_inv_transform = np.linalg.inv(res.transformation)
        # render_frame(
        #     vis, 
            
        #     org_pc_segmented[0],
        #     org_object,
        #     temp_inv_transform,
        #     f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/post_processing_gif/frame_{num_captures:03d}.png"
        # )
        render_frame_option_b_reset(
            vis,
            temp_target,
            org_object,
            temp_inv_transform,
            f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/post_processing_gif/frame_{num_captures:03d}.png"
        )
        num_captures += 1

    print(f"Captured {num_captures} frames for GIF creation.")
        
    images = []
    for i in range(num_captures):
        images.append(imageio.imread(file_location + f"frame_{i:03d}.png"))

    imageio.mimsave(file_location + "Test_registration.gif", images, duration=0.1, loop=0)
    #** Model from realsense capture **
    #Load Object Model
    # source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    #     voxel_size)

    # Global Registration
    # result_ransac = execute_global_registration(source_down, target_down,
    #                                             source_fpfh, target_fpfh,
    #                                             voxel_size)

    # result_ransac_op = execute_global_registration(target_down, source_down,
    #                                             target_fpfh, source_fpfh,
    #                                             voxel_size)

    # print(result_ransac)
    # print(result_ransac_op)
    # draw_registration_result(source_down, target_down, result_ransac.transformation)
    # print(f"RANSAC transformation: {result_ransac.transformation}")
    # print(f"RANSAC op transformation: {result_ransac.transformation}")


    # # Refine Registration
    # result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
    #                                  voxel_size)
    # print(result_icp)
    # draw_registration_result(source, target, result_icp.transformation)


