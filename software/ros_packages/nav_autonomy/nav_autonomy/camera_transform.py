import numpy as np

class Camera:
    def __init__(self, x, y, theta, camera_matrix, dist_coeffs):
        """
        Camera pose + intrinsics.

        Extrinsics (relative to rover):
            x, y   : position (meters)
                     x = forward, y = left
            theta  : yaw (radians)

        Intrinsics:
            camera_matrix : 3x3
            dist_coeffs   : (k1, k2, p1, p2, k3, ...)
        """
        # Extrinsics
        self.x = x
        self.y = y
        self.theta = theta

        # Intrinsics
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # Cache useful intrinsic values
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]

    # -----------------------------
    # OpenCV → rover frame (2D)
    # -----------------------------
    def camera_to_rover_2d(self, tvec):
        x_cam = tvec[0]
        z_cam = tvec[2]

        forward = z_cam
        left = -x_cam

        return np.array([forward, left])

    # -----------------------------
    # Apply camera mounting offset
    # -----------------------------
    def apply_offset(self, point):
        forward, left = point

        dx = forward * np.cos(self.theta) - left * np.sin(self.theta)
        dy = forward * np.sin(self.theta) + left * np.cos(self.theta)

        x_rover = self.x + dx
        y_rover = self.y + dy

        return np.array([x_rover, y_rover])

    # -----------------------------
    # Main function
    # -----------------------------
    def get_target_position(self, tvec):
        point_cam = self.camera_to_rover_2d(tvec)
        return self.apply_offset(point_cam)

    # -----------------------------
    # Intrinsics-based helpers
    # -----------------------------
    def pixel_to_ray(self, u, v):
        """
        Convert pixel (u, v) → normalized camera ray

        Returns direction vector in camera frame
        """
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        # In camera frame: z = 1 forward
        return np.array([x, y, 1.0])

    def pixel_to_bearing(self, u):
        """
        Horizontal angle from image center
        """
        return np.arctan2((u - self.cx), self.fx)

    # -----------------------------
    # Useful helpers
    # -----------------------------
    def get_distance(self, tvec):
        return np.linalg.norm(tvec)

    def get_bearing_camera(self, tvec):
        return np.arctan2(tvec[0], tvec[2])

    def get_bearing_rover(self, tvec):
        return self.get_bearing_camera(tvec) + self.theta