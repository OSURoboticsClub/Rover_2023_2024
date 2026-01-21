import math
from geometry_msgs.msg import PoseStamped
from typing import List

def spiral(
    start_pose: PoseStamped,
    max_radius: float,
    spacing: float,
    points_per_revolution: int = 12
) -> List[PoseStamped]:
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
    
    center_x = start_pose.pose.position.x
    center_y = start_pose.pose.position.y
    
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
        waypoint.header.frame_id = start_pose.header.frame_id
        waypoint.header.stamp = start_pose.header.stamp
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = start_pose.pose.position.z
        
        tangent_angle = theta + math.pi / 2.0
        waypoint.pose.orientation.z = math.sin(tangent_angle / 2.0)
        waypoint.pose.orientation.w = math.cos(tangent_angle / 2.0)
        
        waypoints.append(waypoint)
        theta += theta_step
    
    return waypoints