import rclpy
from rclpy.node import Node









class PathFollower(Node):
        def __init__(self):
                super().__init__('path_folower')


        




def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
        main()