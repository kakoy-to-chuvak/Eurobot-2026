import rclpy
from rclpy.node import Node
import json

try:
    from target_types import TargetType, LiftTarget, SleepTarget
except:
    from route_follower.target_types import TargetType, LiftTarget, SleepTarget

class Parameters:
    def __init__(self, node: Node):
        self.__node__ = node

        self.file = self.create_parameter("route_file", "")



    def create_parameter(self, name: str, value):
        self.__node__.declare_parameter(name, value)
        return self.__node__.get_parameter(name).value

class TargetHandler:
    def __init__(self, target_type: TargetType):
        self.tgt = target_type        
        self.wait = True

    def set_target(self, parameters: dict) -> bool:
        if 'wait' in parameters.keys() and isinstance(parameters['wait'], bool):
            self.wait = parameters['wait']
            parameters.pop('wait')
        return self.tgt.set_target(parameters)
    
    def is_complete(self) -> bool:
        return self.tgt.is_complete()

    def get_progress(self) -> float:
        return self.tgt.get_progress()
    

class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')

        self.parameters = Parameters(self)

        self.target_types: dict[str:TargetType] = {
            "move_lift": LiftTarget(self),
            "sleep": SleepTarget(self),
        }
        
        self.tasks: dict[str:TargetHandler] = {}
        self.get_logger().info(f"Read json file: {self.parameters.file}")
        with open(self.parameters.file, "r") as f:
            self.route = json.load(f)

        if not self.check_json(self.route):
            self.get_logger().error(f"JSON error. Shutdown...")
            raise Exception("JSON Parse error")
        
        self.current_iter = 0

        self.main_timer = self.create_timer(0.1, self.main_cycle)
        
    def main_cycle(self):
        if self.check_tasks():
            return

        try:
            self.new_tasks(self.route[self.current_iter])
            self.current_iter += 1
        except IndexError:
            self.get_logger().info("All tasks completed")
            self.get_logger().info("Shutdown...")
            rclpy.shutdown()

    def new_tasks(self, tasks: dict):
        self.get_logger().info(f"New tasks: {tasks}")
        for name in tasks.keys():
            if not name in self.tasks.keys():
                self.get_logger().info(f"Creating new task <{name}>")
                self.tasks[name] = TargetHandler(self.target_types[name])
            self.tasks[name].set_target(tasks[name])


    def check_json(self, route) -> bool:
        types_set = set(self.target_types.keys())
        if not isinstance(route, list):
            return False
        
        for tgt in route:
            if not isinstance(tgt, dict):
                return False
            
            cur_set = set(tgt.keys())
            if not cur_set.issubset(types_set):
                self.get_logger().error(f"Unknown target type(s): {cur_set.difference(types_set)}")
                return False
        return True


    def check_tasks(self) -> bool:
        waiting = False
        completed_tasks: list[str] = []
        for name in self.tasks.keys():
            task = self.tasks[name]

            if task.wait and not task.is_complete():
                waiting = True

            if task.is_complete():
                self.get_logger().info(f"Task <{name}> completed!")
                completed_tasks.append(name)
            else:
                self.get_logger().debug(f"Task <{name}>: {task.get_progress() * 100:.2f}%")

        for name in completed_tasks:
            self.tasks.pop(name)
        return waiting




def main(args=None):
    rclpy.init(args=args)

    node = RouteFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown...")

    finally:
        try:
            node.destroy_node()
        except:
            pass



if __name__ == "__main__":
    main()