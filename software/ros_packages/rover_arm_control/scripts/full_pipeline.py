import numpy as np
import open3d as o3d
import Test_open3d_v3 as test_o3d
import Visualize_line_segment as vls
import copy
import imageio.v2 as imageio
from concurrent.futures import ThreadPoolExecutor
from scipy.spatial.transform import Rotation as R


def load_capture(capture_path, logger=print):
    """Load an existing capture from disk as an Open3D point cloud.

    Supports Zivid ``.zdf`` files as well as any point-cloud format Open3D can
    read (``.ply``, ``.pcd`` ...). Data is assumed to be in millimetres, which
    is the scale the registration pipeline expects.
    """
    if capture_path.lower().endswith(".zdf"):
        logger(f"Loading Zivid capture: {capture_path}")
        return test_o3d.get_pc_from_zdf(capture_path)

    logger(f"Loading point-cloud capture: {capture_path}")
    pcd = o3d.io.read_point_cloud(capture_path)
    if len(pcd.points) == 0:
        raise ValueError(f"Capture '{capture_path}' contains no points.")
    return pcd


def compute_part_transform(capture_path,
                           cad_mesh_path,
                           voxel_size=50.0,
                           model_sample_points=1000000,
                           num_attempts=48,
                           max_workers=12,
                           logger=print):
    """Register the CAD model against an existing capture.

    This is the headless (no-visualization) core of the ``__main__`` pipeline
    below, packaged for reuse by the ``part_perception`` ROS node.

    All geometry is processed in millimetres. The returned ``camera_to_cad`` is
    expressed in **metres** and follows the frame-transform convention used by
    the publishers (the pose of ``cad_frame`` expressed in the camera frame),
    i.e. ``world_to_cad = world_to_camera @ camera_to_cad``.

    Returns
    -------
    camera_to_cad : (4, 4) np.ndarray
        Pose of the CAD frame in the camera frame, translation in metres.
    best_transformation : (4, 4) np.ndarray
        Raw capture(mm) -> CAD(mm) transform from multi-scale ICP.
    best_fitness : float
        Fitness of the best multi-scale ICP result.
    """
    # --- Load and preprocess the captured cloud -------------------------------
    capture = load_capture(capture_path, logger=logger)
    logger(f"Capture has {len(capture.points)} points.")
    pc_segmented = [capture]

    pc_downs = []
    pc_fpfhs = []
    org_pc_segmented = []
    for i, cluster in enumerate(pc_segmented):
        pc_down, pc_fpfh, org_pc = test_o3d.preprocess_point_cloud(cluster, voxel_size)
        pc_downs.append(pc_down)
        pc_fpfhs.append(pc_fpfh)
        org_pc_segmented.append(org_pc)
        logger(f"Cluster {i} downsampled to {len(pc_down.points)} points.")

    # --- Load and preprocess the CAD model ------------------------------------
    object_mesh = test_o3d.load_model_as_point_cloud(cad_mesh_path, n_points=model_sample_points)
    logger(f"CAD model has {len(object_mesh.points)} points.")
    object_down, object_fpfh, org_object = test_o3d.preprocess_point_cloud(object_mesh, voxel_size)
    logger(f"CAD model downsampled to {len(object_down.points)} points.")

    # --- Parallel global (RANSAC) registration --------------------------------
    logger(f"Running {num_attempts} global registration attempts...")
    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        results = list(ex.map(
            test_o3d.run_parallel_registration,
            range(num_attempts),
            [pc_segmented] * num_attempts,
            [pc_downs] * num_attempts,
            [pc_fpfhs] * num_attempts,
            [object_down] * num_attempts,
            [object_fpfh] * num_attempts,
            [voxel_size] * num_attempts,
        ))

    # --- Multi-scale ICP refinement -------------------------------------------
    device = o3d.core.Device("CUDA:0" if o3d.core.cuda.is_available() else "CPU:0")
    logger(f"Refining with multi-scale ICP on {device}")
    org_pcd_t = o3d.t.geometry.PointCloud.from_legacy(org_pc_segmented[0], device=device)
    org_object_t = o3d.t.geometry.PointCloud.from_legacy(org_object, device=device)

    estimation = o3d.t.pipelines.registration.TransformationEstimationPointToPlane()
    criteria_list = [
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.0001, 0.0001, 20),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 15),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 10),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.0000001, 0.0000001, 5),
    ]
    max_correspondence_distances = o3d.utility.DoubleVector([50, 20, 7.5, 1.5])
    voxel_size_t = o3d.utility.DoubleVector([30, 10, 5, 1])

    best_fitness = -1.0
    best_transformation = None
    for i, res in enumerate(results):
        # Fitness at exactly 0 or 1 means the global registration failed.
        if abs(res.fitness - 1.0) < 1e-9 or abs(res.fitness) < 1e-9:
            continue
        transform_t = o3d.core.Tensor(
            res.transformation, dtype=o3d.core.Dtype.Float32, device=device)
        result_multi = o3d.t.pipelines.registration.multi_scale_icp(
            org_pcd_t, org_object_t, voxel_size_t, criteria_list,
            max_correspondence_distances, transform_t, estimation)
        if result_multi.fitness > best_fitness:
            best_fitness = result_multi.fitness
            best_transformation = result_multi.transformation.cpu().numpy()

    if best_transformation is None:
        raise RuntimeError("Registration failed: no valid global registration result to refine.")

    logger(f"Best multi-scale ICP fitness: {best_fitness}")

    # --- Convert capture->CAD (mm) into camera->cad frame transform (m) --------
    best_transform_inverse = np.linalg.inv(best_transformation)
    camera_to_cad = np.eye(4)
    camera_to_cad[0:3, 0:3] = best_transform_inverse[0:3, 0:3]
    camera_to_cad[0:3, 3] = best_transform_inverse[0:3, 3] / 1000.0

    translation = camera_to_cad[0:3, 3]
    quat = R.from_matrix(camera_to_cad[0:3, 0:3]).as_quat()  # x, y, z, w
    logger(f"camera->cad position (m): {translation}")
    logger(f"camera->cad quaternion (x,y,z,w): {quat}")

    return camera_to_cad, best_transformation, best_fitness


if __name__ == "__main__":
    """
    Welding data is in mm scale so all parameters need to be in mm not m. 
    """
    make_gif = False
    run_line_to_cad = False
    #Load Capture Point Cloud
    zivid_file = "/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/base_plate_orbit_mid_5.zdf"

    #load target point cloud from zdf
    pc_zivid = test_o3d.get_pc_from_zdf(zivid_file)
    pc_segmented = [pc_zivid]
    # pc_segmented = segment_welding_object(pc_zivid, visualize=True, eps = 10.0, min_points = 500, dist_threshold = 2.0, remove_plane=False, cluster = True)

    #load a single mesh-segmented point cloud from ply (for testing)
    # pc_segmented = []
    # pc_segmented.append(o3d.io.read_point_cloud("/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/mid_orbit_3_segment_0.ply"))

    #Process captured point cloud
    pc_downs = []
    pc_fpfhs = []
    org_pc_segmented = []
    voxel_size = 50.0
    for i, cluster in enumerate(pc_segmented):
        o3d.io.write_point_cloud(f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/WUT_Captures/mid_orbit_5_segment_{i}.ply", cluster)
        print(f"Cluster {i} has {len(cluster.points)} points.")
        pc_down, pc_fpfh, org_pc = test_o3d.preprocess_point_cloud(cluster, voxel_size)
        pc_downs.append(pc_down)
        pc_fpfhs.append(pc_fpfh)
        org_pc_segmented.append(org_pc)
        print(f"Cluster {i} downsampled to {len(pc_downs[i].points)} points.")

    #load target object from mesh library and prepare dataset
    object_path = "/home/jn2/college/Cobot_Welding_Masters_Project/meshes/Shortened-25038-WA-44-1.stl"
    object_mesh = test_o3d.load_model_as_point_cloud(object_path, n_points=10000000)
    print(f"object has {len(object_mesh.points)} points.")
    object_down, object_fpfh, org_object = test_o3d.preprocess_point_cloud(object_mesh, voxel_size)
    print(f"object downsampled to {len(object_down.points)} points.")
    # draw_registration_result(object_down, pc_downs[0], np.identity(4))

    print("Starting parallel registration with multiprocessing")
    best_global_transformation = None
    best_global_fitness = -0.01
    print(f"Pc_segmented length: {len(pc_segmented)}, pc_downs length: {len(pc_downs)}, org_pc_segmented length: {len(org_pc_segmented)}, pc_fpfhs length: {len(pc_fpfhs)}")
    with ThreadPoolExecutor(max_workers=12) as ex:
        results = list(ex.map(test_o3d.run_parallel_registration, range(48), [pc_segmented]*48, [pc_downs]*48, [pc_fpfhs]*48, [object_down]*48, [object_fpfh]*48, [voxel_size]*48))

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

    #Run Multi-Scale ICP for each global registration result
    best_fitness_multi = -1.0
    best_transformation_multi = None
    best_idx_multi = 0
    iteration_multi = []
    best_iteration_transforms_multi = None


    for i, res in enumerate(results):
        #Results with fitness close to 0 or 1 mean the global registration failed. 
        if abs(res.fitness-1.0) < 0.000000001 or abs(res.fitness) < 0.000000001:
            continue
        transform_t = o3d.core.Tensor(
            res.transformation, 
            dtype=o3d.core.Dtype.Float32, 
            device=device
        )
        #Setup Callback
        iteration_multi = []
        callback_after_iteration_multi = test_o3d.make_transform_icp_callback(iteration_multi)

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
            callback_after_iteration_multi
        )
        print("Multi-Scale ICP Result: Iteration: {}, Fitness: {}, Inlier RMSE: {}".format(
            i,
            result_multi.fitness,
            result_multi.inlier_rmse,
        ))
        # print(f"Transform for run {i}: {iteration_multi}")
        #Update best fitness and transform 
        if result_multi.fitness > best_fitness_multi:
            best_fitness_multi = result_multi.fitness
            best_transformation_multi = result_multi.transformation.cpu().numpy()
            best_idx_multi = i
            best_iteration_transforms_multi = iteration_multi

    #Show Results
    best_transform_inverse = np.linalg.inv(best_transformation_multi)
    print(f"Best fitness multi-scale: {best_fitness_multi}")
    print(f"Best transformation multi-scale: {best_transformation_multi}")
    print(f"Inverted transform {np.linalg.inv(best_transformation_multi)}")
    test_o3d.draw_registration_result(org_pc_segmented[0], org_object, best_transformation_multi)
    # print(f"Iteration transforms: {best_iteration_transforms_multi}")

    translation = best_transform_inverse[0:3, 3] / 1000.0

    # --- Rotation ---
    rotation_matrix = best_transform_inverse[0:3, 0:3]
    rotation = R.from_matrix(rotation_matrix)
    quat = rotation.as_quat()  # x, y, z, w

    print("Position (m):", translation)
    print("Quaternion (x,y,z,w):", quat)

    #Create Gif of Transforms
    if make_gif:
        # vis = setup_visualizer()
        temp_target = copy.deepcopy(org_object)
        temp_target.paint_uniform_color([0, 0.651, 0.929])  
        temp_source = copy.deepcopy(org_pc_segmented[0])
        temp_source.paint_uniform_color([1, 0.706, 0])
        # vis.add_geometry(temp_target)
        # vis.add_geometry(temp_source)
        # set_fixed_view(vis)
        # vis.reset_view_point(False)
        vis = test_o3d.setup_vis_with_fixed_target(temp_source, temp_target)

        #Capture Global Registration Results
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
            test_o3d.render_frame_option_b_reset(
                vis,
                temp_target,
                org_object,
                temp_inv_transform,
                f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/post_processing_gif/frame_{num_captures:03d}.png"
            )
            num_captures += 1

        print(f"Captured {num_captures} frames for GIF creation.")

        # Create GIF from captured frames    
        images = []
        for i in range(num_captures):
            images.append(imageio.imread(file_location + f"frame_{i:03d}.png"))

        imageio.mimsave(file_location + "Test_registration.gif", images, fps=1, loop=0)


        #Gif For Best ICP
        num_captures=0
        temp_target = copy.deepcopy(org_object)
        temp_target.paint_uniform_color([0, 0.651, 0.929])  
        temp_source = copy.deepcopy(org_pc_segmented[0])
        temp_source.paint_uniform_color([1, 0.706, 0])
        vis2 = test_o3d.setup_vis_with_fixed_target(temp_source, temp_target)

        for i, trans in enumerate(best_iteration_transforms_multi):
            temp_trans = trans
            temp_trans = np.linalg.inv(trans)
            test_o3d.render_frame_option_b_reset(
                vis2,
                temp_target,
                org_object,
                temp_trans,
                f"/home/jn2/college/Cobot_Welding_Masters_Project/Captures/post_processing_gif/icp_frame_{num_captures:03d}.png"
            )
            num_captures += 1
        print(f"Captured {num_captures} frames for ICP GIF creation.")
        # Create GIF from captured frames    
        images = []
        for i in range(num_captures):
            images.append(imageio.imread(file_location + f"icp_frame_{i:03d}.png"))

        imageio.mimsave(file_location + "Test_icp_registration.gif", images, fps=10, loop=0)


        

    #Weld seam visualization
    
    #read tool path
    segments = vls.read_segments_from_file("/home/jn2/college/Cobot_Welding_Masters_Project/data/1001.dat")
    for p1, p2 in segments:
        print(f"Segment from {p1} to {p2}")

    # Create coordinate frame for visualization
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=200.0,   # scale to your scene
    origin=[0, 0, 0]
    )

    #create line segments for visualization
    line_set = vls.lineset_from_segments(segments)
    # o3d.visualization.draw_geometries([line_set, origin])

    #create known transform to align tool path
    transform = np.array([
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    # test_o3d.draw_registration_result(line_set, org_object, transform)

    if run_line_to_cad:
        #Transform object to tool path frame (Should do the opposite)
        clone_object = copy.deepcopy(org_object)
        clone_object.transform(transform)

        #Find Close Points
        dist_threshold = 2 #distance [mm]
        points = np.asarray(clone_object.points)
        mask = np.zeros(len(points), dtype=bool)

        for a, b in segments:
            d = vls.point_to_line_segment(points, np.array(a), np.array(b))
            mask |= d < dist_threshold

        #Create new point clouds
        idxs = np.where(mask)[0]
        near_pcd = clone_object.select_by_index(idxs)
        removed_pcd = clone_object.select_by_index(idxs, invert=True)
        print(f"Found {len(near_pcd.points)} points near the line segments.")

        #Show Results
        o3d.visualization.draw_geometries([near_pcd])
        test_o3d.draw_registration_result(near_pcd, removed_pcd, np.eye(4))

    #Find Closest points in Captured Point Cloud

    test_o3d.draw_registration_result(org_pc_segmented[0], line_set, transform @ best_transformation_multi)
    clone_captured = copy.deepcopy(org_pc_segmented[0])
    clone_captured.transform(transform @best_transformation_multi)
    # test_o3d.draw_registration_result(clone_captured, org_object, np.eye(4))
    dist_threshold = 2  # distance [mm]
    points_captured = np.asarray(clone_captured.points)
    mask_captured = np.zeros(len(points_captured), dtype=bool)

    for a, b in segments:
        d = vls.point_to_line_segment(points_captured, np.array(a), np.array(b))
        mask_captured |= d < dist_threshold

    #Create new point clouds
    idxs = np.where(mask_captured)[0]
    near_pcd_captured = org_pc_segmented[0].select_by_index(idxs)
    removed_pcd_captured = org_pc_segmented[0].select_by_index(idxs, invert=True)
    print(f"Found {len(near_pcd_captured.points)} points near the line segments in captured point cloud.")
    # o3d.visualization.draw_geometries([near_pcd_captured])
    test_o3d.draw_registration_result(near_pcd_captured, removed_pcd_captured, np.eye(4))
    test_o3d.draw_registration_result(near_pcd_captured, org_object, best_transformation_multi)

    idx = vls.closest_indices_along_segments(clone_captured, segments[1:-1], spacing=10)
    near_lin_pcd = clone_captured.select_by_index(idx)
    removed_lin_pcd = clone_captured.select_by_index(idx, invert=True)

    # o3d.visualization.draw_geometries([near_lin_pcd])
    # test_o3d.draw_registration_result(near_lin_pcd, removed_lin_pcd, np.eye(4))
    # test_o3d.draw_registration_result(near_lin_pcd, org_object, transform)

    near_line_set = vls.indices_to_lineset(clone_captured, idx, [1, 0.706, 0])
    # o3d.visualization.draw_geometries([
    #     org_object,
    #     near_line_set
    # ])
    test_o3d.draw_registration_result(near_line_set, org_object, transform)