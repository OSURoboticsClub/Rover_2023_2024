from enum import Enum
import math
from geometry_msgs.msg import PoseStamped
from typing import List

def spiral(center: PoseStamped, max_radius: float, spacing: float, points_per_revolution: int = 12) -> List[PoseStamped]:
    """
    Generate an Archimedean spiral search pattern.
    
    Args:
        start_pose: Starting pose (center of spiral)
        max_radius: Maximum radius of spiral in meters
        spacing: Distance between spiral loops in meters
        points_per_revolution: Number of waypoints per revolution
    
    Returns:
        List of PoseStamped waypoints forming the spiral
    """
    waypoints = []
    
    center_x = center.pose.position.x
    center_y = center.pose.position.y
    
    a = spacing / (2.0 * math.pi)
    theta_max = max_radius / a
    theta_step = (2.0 * math.pi) / points_per_revolution        # KRJ TODO: do we want consistently spaces points instead?
    
    theta = 0.0
    while theta <= theta_max:
        r = a * theta
        if r > max_radius:
            break
        
        x = center_x + r * math.cos(theta)
        y = center_y + r * math.sin(theta)
        
        waypoint = PoseStamped()
        waypoint.header.frame_id = center.header.frame_id
        waypoint.header.stamp = center.header.stamp
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = center.pose.position.z
        
        tangent_angle = theta + math.pi / 2.0
        waypoint.pose.orientation.z = math.sin(tangent_angle / 2.0)
        waypoint.pose.orientation.w = math.cos(tangent_angle / 2.0)
        
        waypoints.append(waypoint)
        theta += theta_step
    
    return waypoints


# corners[0] is start point, then clockwise.
# longest edge defines sweep axis, shorter edge defines spacing axis.
# p1 ---------- p2     p0 ---------- p1
# |              |     |              |
# |              |     |              |
# p0 ---------- p3     p3 ---------- p2
def lawnmower(corners: List[PoseStamped], spacing: float, step_size: float) -> List[PoseStamped]:

    if len(corners) < 4:
        return []

    p0 = corners[0].pose.position
    p1 = corners[1].pose.position
    p2 = corners[2].pose.position
    p3 = corners[3].pose.position

    sq_dist_01 = (p1.x - p0.x)**2 + (p1.y - p0.y)**2
    sq_dist_03 = (p3.x - p0.x)**2 + (p3.y - p0.y)**2

    if sq_dist_01 >= sq_dist_03:
        sweep_v = (p1.x - p0.x, p1.y - p0.y)
        space_v = (p3.x - p0.x, p3.y - p0.y) # Spacing goes toward p3
        sweep_len = math.sqrt(sq_dist_01)
        space_len = math.sqrt(sq_dist_03)
    else:
        sweep_v = (p3.x - p0.x, p3.y - p0.y)
        space_v = (p1.x - p0.x, p1.y - p0.y) # Spacing goes toward p1
        sweep_len = math.sqrt(sq_dist_03)
        space_len = math.sqrt(sq_dist_01)

    ux = sweep_v[0] / sweep_len
    uy = sweep_v[1] / sweep_len
    nx = space_v[0] / space_len
    ny = space_v[1] / space_len

    waypoints = []
    num_lanes = int(space_len / spacing) + 1
    num_steps = int(sweep_len / step_size) + 1

    for i in range(num_lanes):
        lane_offset = i * spacing
        reverse = (i % 2 != 0)
        for j in range(num_steps):
            d = (sweep_len - (j * step_size)) if reverse else (j * step_size)
            
            wp = PoseStamped()
            wp.header = corners[0].header
            wp.header.frame_id = corners[0].header.frame_id
            # Start at p0, offset by lane index, then move along sweep line
            wp.pose.position.x = p0.x + (lane_offset * nx) + (d * ux)
            wp.pose.position.y = p0.y + (lane_offset * ny) + (d * uy)
            wp.pose.orientation.w = 1.0
            waypoints.append(wp)

    def fix_orientation(waypoints: List[PoseStamped]) -> List[PoseStamped]:
        for i in range(len(waypoints) - 1):
            curr_wp = waypoints[i]
            next_wp = waypoints[i + 1]
            dx = next_wp.pose.position.x - curr_wp.pose.position.x
            dy = next_wp.pose.position.y - curr_wp.pose.position.y
            yaw = math.atan2(dy, dx)
            curr_wp.pose.orientation.z = math.sin(yaw / 2.0)
            curr_wp.pose.orientation.w = math.cos(yaw / 2.0)
            
        if len(waypoints) >= 2:
            waypoints[-1].pose.orientation = waypoints[-2].pose.orientation

        return waypoints
    
    return fix_orientation(waypoints)
