import rclpy
import camera_gimbal_controller.gimbal_control as gc

def main(args=None):
    rclpy.init()

    node = gc.GimbalControlNode(topic="/tower_gimbal/control")

    while(True):
        node.update()
        rclpy.spin_once(node, timeout_sec=.05)


if __name__ == "__main__":
    main()