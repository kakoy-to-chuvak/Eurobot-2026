import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from target_types import TargetType, LiftTarget, SleepTarget
except:
    from route_follower.target_types import TargetType, LiftTarget, SleepTarget, NavTarget, ServoTarget

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
        try:
            self.wait = bool(parameters.get('wait', True))
        except:
            self.wait = True

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
            "move_lift":  LiftTarget(self),
            "sleep":      SleepTarget(self),
            "nav_tgt":    NavTarget(self),
            "move_servo": ServoTarget(self),
        }
        
        self.tasks: dict[str:TargetHandler] = {}
        self.get_logger().info(f"Read json file: {self.parameters.file}")
        with open(self.parameters.file, "r") as f:
            try:
                self.route = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Error on parsng JSON: {str(e)}")
                self.get_logger().info("Shutdown...")
                rclpy.shutdown()
                exit(-1)

        if not self.check_json(self.route):
            self.get_logger().error(f"Error on checking JSON")
            self.get_logger().info("Shutdown...")
            rclpy.shutdown()
            exit(-1)


        self.current_iter = 0
        self.main_timer = self.create_timer(0.1, self.main_cycle)
        self.state_pub = self.create_publisher(String, "/pwb/tasks", 10)
        self.start_service = self.create_service(Trigger, '/route_follower/start', self.start_callback)
        self.started = False
    
    def start_callback(self, request, response):
        if self.started:
            response.success = False
            response.message = "Route already started"
            return response
        self.started = True
        self.get_logger().info("Route execution started by service call")
        response.success = True
        response.message = "Route started"
        return response
        
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
                self.tasks[name] = TargetHandler(self.target_types[name])
            self.get_logger().info(f"Setting new task <{name}> wait: {tasks[name].get('wait', True)}")
            if not self.tasks[name].set_target(tasks[name]):
                self.get_logger().error(f"Failed to set target <{name}>")
                self.get_logger().info("Shutdown...")
                rclpy.shutdown()
                return


    def check_json(self, route) -> bool:
        types_set = set(self.target_types.keys())
        if not isinstance(route, list):
            self.get_logger().error(f"Wrong JSON reference")
            return False
        
        for tgts in route:
            if not isinstance(tgts, dict):
                self.get_logger().error(f"Wrong JSON reference")
                return False
            
            cur_set = set(tgts.keys())
            if not cur_set.issubset(types_set):
                self.get_logger().error(f"Unknown target type(s): {cur_set.difference(types_set)}")
                return False
        
        return True


    def check_tasks(self) -> bool:
        waiting = False
        current_tasks = {}
        completed_tasks: list[str] = []
        for name in self.tasks.keys():
            task: TargetHandler = self.tasks[name]
            complete = task.is_complete()
            progress = task.get_progress()
            
            if task.wait and not complete:
                waiting = True

            if complete:
                self.get_logger().info(f"Task <{name}> completed!")
                completed_tasks.append(name)
            else:
                self.get_logger().debug(f"Task <{name}>: {progress * 100:.2f}%")
                current_tasks[name] = {
                    "wait": task.wait,
                    "progress": progress
                }
        
        current_tasks['completed'] = completed_tasks
        msg = String()
        msg.data = str(current_tasks)
        self.state_pub.publish(msg)

        # delete completed tasks
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