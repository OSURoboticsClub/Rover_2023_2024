import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class CostmapSurface(Node):

    def __init__(self):
        super().__init__("costmap_surface")

        self.sub = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.costmap_callback,
            10
        )

        self.pub = self.create_publisher(
            Marker,
            "/costmap_surface_marker",
            10
        )

        # Tunable parameters
        self.height_scale = 0.5   # meters max height
        self.cost_threshold = 0   # ignore unknown/low if needed
        self.step = 1             # increase to downsample (2, 3, etc.)

    # ----------------------------------
    # Costmap Callback
    # ----------------------------------

    def costmap_callback(self, msg):

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution

        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = msg.data

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "costmap_surface"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        # Required but not used for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.pose.orientation.w = 1.0

        marker.points = []
        marker.colors = []

        triangle_count = 0

        for y in range(0, height - 1, self.step):
            for x in range(0, width - 1, self.step):

                idx = y * width + x
                cost = data[idx]

                if cost < self.cost_threshold or cost < 0:
                    continue

                # Heights for 4 corners
                def get_z(ix, iy):
                    i = iy * width + ix
                    c = data[i]
                    if c < 0:
                        return 0.0
                    return (c / 100.0) * self.height_scale

                z0 = get_z(x, y)
                z1 = get_z(x + 1, y)
                z2 = get_z(x + 1, y + 1)
                z3 = get_z(x, y + 1)

                # World coordinates
                def make_point(ix, iy, z):
                    p = Point()
                    p.x = ox + ix * res
                    p.y = oy + iy * res
                    p.z = z
                    return p

                p0 = make_point(x, y, z0)
                p1 = make_point(x + 1, y, z1)
                p2 = make_point(x + 1, y + 1, z2)
                p3 = make_point(x, y + 1, z3)

                # Color mapping (green → red)
                def make_color(z):
                    ratio = min(max(z / self.height_scale, 0.0), 1.0)
                    c = ColorRGBA()
                    c.r = ratio
                    c.g = 1.0 - ratio
                    c.b = 0.0
                    c.a = 1.0
                    return c

                c0 = make_color(z0)
                c1 = make_color(z1)
                c2 = make_color(z2)
                c3 = make_color(z3)

                # Triangle 1: p0, p1, p2
                marker.points.extend([p0, p1, p2])
                marker.colors.extend([c0, c1, c2])

                # Triangle 2: p0, p2, p3
                marker.points.extend([p0, p2, p3])
                marker.colors.extend([c0, c2, c3])

                triangle_count += 2

        self.pub.publish(marker)

        self.get_logger().info(
            f"Published surface: {triangle_count} triangles, {len(marker.points)} points"
        )


# ----------------------------------
# Main
# ----------------------------------

def main():
    rclpy.init()
    node = CostmapSurface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()