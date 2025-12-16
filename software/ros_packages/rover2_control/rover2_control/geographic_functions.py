from geographiclib.geodesic import Geodesic
from dataclasses import dataclass
import math

@dataclass
class Location:
    latitude: float
    longitude: float

def get_target_heading(rover_position, target):
    geod = Geodesic.WGS84
    lat1 = rover_position.latitude
    lon1 = rover_position.longitude
    lat2 = target.latitude
    lon2 = target.longitude
    result = geod.Inverse(lat1, lon1, lat2, lon2)
    return result['azi1'] # dont ask me why

def get_distance_to_location(rover_position, target):
    geod = Geodesic.WGS84
    lat1 = rover_position.latitude
    lon1 = rover_position.longitude
    lat2 = target.latitude
    lon2 = target.longitude
    result = geod.Inverse(lat1, lon1, lat2, lon2)
    return result['s12'] * 3.28084  # Convert meters to feet because this is America

def compute_curvature(rover_position, target, heading_error):
    dist_to_target = get_distance_to_location(rover_position, target)

    # Compute lateral error (y)
    heading_error = math.radians(heading_error)  # Convert to radians
    y = dist_to_target * math.sin(heading_error)  # Perpendicular distance

    # Compute curvature
    if dist_to_target == 0:
        return 0  # Prevent division by zero
    curvature = (2 * y) / (dist_to_target ** 2)
    return curvature

def get_points_along_line(rover_position, waypoint_destination, target_heading, step_feet=15):
    """
    Moves along the geodesic path from (lat1, lon1) to (lat2, lon2) 
    in steps of `step_feet`, returning a list of coordinates
    """
    total_distance = get_distance_to_location(rover_position, waypoint_destination)
    geod = Geodesic.WGS84
    lat1 = rover_position.latitude
    lon1 = rover_position.longitude

    subpoints = []  # Exclude start and end positions
    #self.get_logger().info("Distance: " + str(total_distance))

    for i in range(1, 100):
        traveled_distance = i * step_feet
        #self.get_logger().info("Traveled: " + str(traveled_distance))
        if traveled_distance >= total_distance - step_feet/2.0:
            break
        new_pos = geod.Direct(lat1, lon1, target_heading, traveled_distance * 1/3.28084)
        subpoints.append(Location(new_pos['lat2'], new_pos['lon2']))
    return subpoints
    #msg = "subpoints;" + json.dumps([asdict(loc) for loc in self.subpoints])
    #self.publish_log_msg(msg)