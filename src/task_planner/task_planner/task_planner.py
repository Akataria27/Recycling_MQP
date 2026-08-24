
import rclpy
from rclpy.node import Node


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')
        


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
