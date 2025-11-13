import re

import rclpy
import rclpy.qos
import rclpy.executors
import rclpy.logging
import rclpy.node
import geometry_msgs.msg

import threading
import cooking_zoo

from rclpy.callback_groups import ReentrantCallbackGroup

from dhtt_msgs.msg import Resource
from dhtt_msgs.srv import ResourceRequest

from dhtt_cooking_msgs.msg import CookingAction, CookingAgent, CookingObservation, CookingObject, CookingRecipe, CookingTypes
from dhtt_cooking_msgs.srv import CookingRequest

from cooking_zoo.cooking_agents.base_agent import BaseAgent
from cooking_zoo.environment.cooking_env import CookingEnvironment as CookingZooEnvironment, parallel_env
from cooking_zoo.cooking_world.abstract_classes import Object as cooking_zoo_Object, \
    StaticObject as cooking_zoo_StaticObject
from cooking_zoo.cooking_world.world_objects import Agent as cooking_zoo_Agent
from cooking_zoo.cooking_book.recipe import RecipeNode as cooking_zoo_RecipeNode

from typing import List, Tuple, Dict

DEFAULT_PLAYER_NAME = CookingAction.DEFAULT_PLAYER_NAME  # usually player_0 this is from cooking_zoo cooking_env.py
DEFAULT_AGENT_ID = CookingAction.DEFAULT_AGENT_ID  # usually agent-1. This is the name given to the actual agent object

resource_type_dict = {
    cooking_zoo.cooking_world.world_objects.Counter: 4, # COUNTER=4
    cooking_zoo.cooking_world.world_objects.Cutboard: 5, # CUT_BOARD=5
    cooking_zoo.cooking_world.world_objects.Toaster: 6, # TOASTER=6
    cooking_zoo.cooking_world.world_objects.Pot: 7, # POT=7
    cooking_zoo.cooking_world.world_objects.Blender: 8, # BLENDER=8
    cooking_zoo.cooking_world.world_objects.Pan: 9, # PAN=9
    cooking_zoo.cooking_world.world_objects.PlateDispenser: 10, # PLATE_DISPENSER=10
    cooking_zoo.cooking_world.world_objects.BreadDispenser: 11, # BREAD_DISPENSER=11
    cooking_zoo.cooking_world.world_objects.PastaDispenser: 12, # PASTA_DISPENSER=12
    cooking_zoo.cooking_world.world_objects.IceDispenser: 13, # ICE_DISPENSER=13
    cooking_zoo.cooking_world.world_objects.OnionDispenser: 14, # ONION_DISPENSER=14
    cooking_zoo.cooking_world.world_objects.EggDispenser: 15, # EGG_DISPENSER=15
    cooking_zoo.cooking_world.world_objects.BananaDispenser: 16, # BANANA_DISPENSER=16
    cooking_zoo.cooking_world.world_objects.StrawberryDispenser: 17, # STRAWBERRY_DISPENSER=17
    cooking_zoo.cooking_world.world_objects.TomatoDispenser: 18, # TOMATO_DISPENSER=18
    cooking_zoo.cooking_world.world_objects.LettuceDispenser: 19, # LETTUCE_DISPENSER=19
    cooking_zoo.cooking_world.world_objects.AbsorbingDeliversquare: 20, # ABSORBING_DELIVER_SQUARE=20
    "Counter": 4,
    "Cutboard": 5,
    "Toaster": 6,
    "Pot": 7,
    "Blender": 8,
    "Pan": 9,
    "PlateDispenser": 10,
    "BreadDispenser": 11,
    "PastaDispenser": 12,
    "IceDispenser": 13,
    "OnionDispenser": 14,
    "EggDispenser": 15,
    "BananaDispenser": 16,
    "StrawberryDispenser": 17,
    "TomatoDispenser": 18,
    "LettuceDispenser": 19,
    "AbsorbingDeliversquare": 20,
}

dynamic_resource_type_dict = {
    "Plate": 21,
    "Bread": 22,
    "Pasta": 23,
    "Ice": 24,
    "Onion": 25,
    "Egg": 26,
    "Banana": 27,
    "Strawberry": 28, 
    "Tomato": 29,
    "Lettuce": 30,
    "BreadChopped": 31, 
    "BreadToasted": 32,
    "PastaCooked": 33, 
    "OnionChopped": 34,
    "EggFried": 35,
    "BananaChopped": 36, 
    "TomatoChopped": 37,
    "TomatoFried": 38,
    "LettuceChoppped": 39,
    "Smoothie": 40,
}

class CookingNode(rclpy.node.Node):
    """
    Class for handling requests to the cooking_zoo environment

    Starts up ROS **services** and callbacks and keeps an internal instance of the cooking_zoo environment.

    For tight control, runs as a synchronous loop: Upon receiving a service request, steps the environment as many
    times as needed before returning. For some actions, this may be one environment tick, for others it may be several.
    Without a service request, the environment will not tick. To 'busy wait' in the environment, send a series of NO_OP
    actions.
    """

    def __init__(self, ex):
        super().__init__('cooking_zoo')

        self.cooking_environment = CookingEnvironment()
        
        self.qos = rclpy.qos.QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_ALL)

        self.cooking_server = self.create_service(CookingRequest, 'Cooking_Server', self.cooking_request_callback, qos_profile=self.qos)
        self.cooking_observation_publisher = self.create_publisher(CookingObservation, 'Cooking_Observations', 10)
        self.client = self.create_client(ResourceRequest, "/resource_service")

        self.service_lock = threading.Lock()

        self.actions_taken: list[CookingAction] = []

        self.thread = None
        self.detectors = []
        self.my_ex = ex

        self.tick = threading.Condition()

        if self.cooking_environment:
            obs = self._prepare_observation_msg()
            self.cooking_observation_publisher.publish(obs)
            self.get_logger().info('Published initial observation')
            self.get_logger().debug(f'Initial obs: {obs}')

    def cooking_request_callback(self, request: CookingRequest.Request,
                                 response: CookingRequest.Response) -> CookingRequest.Response:
        # acquire a lock to prevent multithreading issue with the callback
        with self.service_lock:
            
            to_ret = ""
            
            if request.super_action == CookingRequest.Request.START:
                self.get_logger().info('Got start request')
                self.cooking_environment.start()
                self.dispatch_resource_update()
            elif request.super_action == CookingRequest.Request.STOP:
                self.get_logger().info('Got stop request')
                self.cooking_environment.stop()
            elif request.super_action == CookingRequest.Request.RESET:
                self.get_logger().info('Got reset request')
                self.cooking_environment.reset()
                # self.dispatch_resource_update()
            elif request.super_action == CookingRequest.Request.ACTION:
                self.get_logger().info('Got action request')
                if request.action.player_name == "":
                    self.get_logger().error('Bad action request: No player name')
                    to_ret = "Bad action request: No player name"
                else:
                    self.get_logger().info (
                        f'Resolving action request with: player {request.action.player_name}, action_type {request.action.action_type}, params {request.action.params}')
                    self.cooking_environment.resolve_dhtt_action(request, response)

                    if response.error_msg == "":
                        self.actions_taken.append(request.action)
                    
                        if request.action.action_type == CookingAction.EXECUTE_ACTION_ARM1 or request.action.action_type == CookingAction.EXECUTE_ACTION_ARM2:
                            self.dispatch_detector_in_front_of_agent()
                self.tick.acquire()
                self.tick.notify_all()
                self.tick.release()
            elif request.super_action == CookingRequest.Request.OBSERVE:
                self.get_logger().info('Got observe request')
                # self.get_logger().info('Published observation')
                # self.get_logger().debug(f'Published observation, header: {obs.head}')
            else:
                to_ret = "No valid super_action"

            # if self.cooking_environment:
            #     self.get_logger().debug(f'Published observation, header: {obs.head}')

            obs = self._prepare_observation_msg()
            self.cooking_observation_publisher.publish(obs)
            response.error_msg += to_ret
            response.success = response.error_msg == ""

            return response

    def _prepare_observation_msg(self) -> CookingObservation:
        obs = self.cooking_environment.get_observation_msg()
        obs.actions = self.actions_taken
        return obs
    
    def dispatch_resource_update(self):
        if self.thread:
            self.thread.join()

        self.get_logger().info("Dispatching resource thread.")

        self.thread = threading.Thread(target=self.register_resources_on_server, args=[])
        self.thread.start()

    def dispatch_detector_in_front_of_agent(self):
        self.get_logger().warn("Dispatching detector for square in front of agent")

        location = self.get_location_in_front()

        self.detectors.append(threading.Thread(target=self.register_resource_when_changed, args=[location]))
        self.detectors[-1].daemon = True # detach thread so we don't have to clean up resources
        self.detectors[-1].start()

        # self.get_logger().warn("Thread dispatched")

    def register_resource_when_changed(self, location):
        # self.get_logger().warn(f'{location}')

        obj_type = resource_type_dict[self.get_static_at_location(location).object_type]

        # self.get_logger().warn(f'{obj_type}')

        # only do this for static objects that cook their items
        if obj_type == CookingTypes.TOASTER or obj_type == CookingTypes.POT or obj_type == CookingTypes.BLENDER or obj_type == CookingTypes.PAN:
        
            # self.get_logger().warn("waiting for change in env")
            self.detect_change_at_location(location)

            # self.get_logger().warn("Found change registering new resource!")

            self.register_resources_at_location_on_server(location)

    def get_location_in_front(self):
        world_agents = self.cooking_environment.env.unwrapped.world.agents
        agents = [self.cooking_environment._agent_to_agent_msg(obj) for obj in world_agents]

        location = agents[0].object_members.location

        if agents[0].orientation == 1:
            location.x -= 1
        elif agents[0].orientation == 2:
            location.x += 1
        elif agents[0].orientation == 3:
            location.y += 1
        else:  
            location.y -= 1

        return location

    def get_dynamic_at_location(self, location):

        to_ret = []
        world_objects = self.cooking_environment.env.unwrapped.world.world_objects

        for obj_list in world_objects.values():
            for obj in obj_list:

                if obj.location[0] == location.x and obj.location[1] == location.y \
                        and not type(obj) in resource_type_dict.keys():
                    to_ret.append(obj)
        
        return to_ret
    
    def get_static_at_location(self, location):
        to_ret = None
        world_objects = self.cooking_environment.env.unwrapped.world.world_objects

        for obj_list in world_objects.values():
            for obj in obj_list:

                if obj.location[0] == location.x and obj.location[1] == location.y \
                        and type(obj) in resource_type_dict.keys():
                    to_ret = obj
        
        return self.cooking_environment._object_to_object_msg(to_ret)
    
    # loops until a change in the dynamic objects at a location is detected
    def detect_change_at_location(self, location):
        initial_state = [ self.cooking_environment._object_to_object_msg(loc) for loc in self.get_dynamic_at_location(location) ]

        loop = True

        while loop:
            # only loop on ticks
            self.tick.acquire()
            self.tick.wait()

            new_state = [ self.cooking_environment._object_to_object_msg(loc) for loc in self.get_dynamic_at_location(location) ]

            # self.get_logger().error("No change at location!")

            # if new state is the same as the old state then we keep looping
            for i in new_state:
                comparison = None

                for j in initial_state:
                    if j.world_id == i.world_id:
                        comparison = j
                        break

                if comparison == None or \
                        comparison.object_type != i.object_type: 
                    loop = False
                    break

                for a in comparison.physical_state:
                    if not a in i.physical_state:
                        loop = False
                        break
            
            self.tick.release()

    def register_resources_at_location_on_server(self, location):
        with self.service_lock:

            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Could not contact resource service")

                raise RuntimeError("Could not contact resource service")

            # add the resources from the environment

            req = ResourceRequest.Request()
            req.type = ResourceRequest.Request.ADD

            for obj in self.get_dynamic_at_location(location):
                n_resource = Resource()

                cooking_obj = self.cooking_environment._object_to_object_msg(obj)

                n_resource.name = cooking_obj.object_type

                relevant_condition = ""

                for cond in cooking_obj.physical_state:
                    if cond == "Chopped":
                        relevant_condition = cond
                    elif cond == "Toasted" or cond == "Cooked" or cond == "Fried":
                        relevant_condition = cond
                        break

                n_resource.name += relevant_condition

                n_resource.type = dynamic_resource_type_dict[n_resource.name]

                n_resource.name +=  "_" + str(cooking_obj.world_id)
                n_resource.channel = 0
                n_resource.locked = False
                n_resource.owners = 0

                req.to_modify._resource_state.append(n_resource)

            # send request to the dhtt server
            self.client.call_async(req)
    
    def register_resources_on_server(self):
        with self.service_lock:

            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("Could not contact resource service")

                raise RuntimeError("Could not contact resource service")

            # add the resources from the environment
            world_objects = self.cooking_environment.env.unwrapped.world.world_objects

            req = ResourceRequest.Request()
            req.type = ResourceRequest.Request.ADD

            for obj_list in world_objects.values():
                for obj in obj_list:

                    if ( obj.location[0] == 0 and obj.location[1] == 0 ) or \
                        ( obj.location[0] == 8 and obj.location[1] == 0 ) or \
                        ( obj.location[0] == 0 and obj.location[1] == 8 ) or \
                        ( obj.location[0] == 8 and obj.location[1] == 8 ):
                        continue

                    if type(obj) in resource_type_dict.keys():
                        n_resource = Resource()

                        n_resource.name = f'{type(obj).__name__}_{obj.unique_id}'
                        n_resource.type = resource_type_dict[type(obj)]
                        n_resource.channel = 0
                        n_resource.locked = False
                        n_resource.owners = 0

                        req.to_modify._resource_state.append(n_resource)

            # send request to the dhtt server
            self.client.call_async(req)

            # what if I just don't block for the response
            # self.my_ex.spin_until_future_complete(future=response_future)
            # res = response_future.result()

            # # throw exception if this did not succeed
            # if not res.success:
            #     self.get_logger().error("Failed to register resources on the server")

            #     raise RuntimeError("Failed to register resources on the server")

            # self.get_logger().fatal("Response received")
        return


class CookingEnvironment:
    """
    Sets up a managed instance of the cooking_zoo environment

    Bridges the gap between ROS service calls and stepping the cooking_zoo environment. ROS service calls determine
    what action the environment should take, including starting, resetting, and the actual agent actions.

    Also gets the latest observation and packages as a ROS message.
    """

    class MyAgent(BaseAgent):
        """
        Pulls some helpful functions from the cooking_zoo BaseAgent. Only really used for MoveTo.
        """

        def __init__(self, name=DEFAULT_AGENT_ID):
            super().__init__("no_recipe", name)

        def step(self, observation) -> int:
            # Implement abstract method, even though we do not want to use it here
            raise NotImplementedError

        # def walk_to_location(self, location, observation):
        #     if not self.reachable(self.location, location, observation):
        #         print("Not reachable")
        #         return 0
        #     else:
        #         return super().walk_to_location(location, observation)

    def __init__(self):
        self._init_cooking_zoo()

        self.env: CookingZooEnvironment
        self.observations: dict
        self.infos: dict
        self.terminations: dict
        self.truncations: dict
        self.env, self.observations, self.infos, self.terminations, self.truncations = self._init_cooking_zoo()

        # self.last_reward = None
        # self.cumulative_reward = 0

        self.my_agent = self.MyAgent()

    def start(self):
        """Initialize the cooking_zoo environment. Typically this is called in the constructor."""
        self.env, self.observations, self.infos, self.terminations, self.truncations = self._init_cooking_zoo()

    def stop(self):
        self.env.close()

    def reset(self):
        self.env.reset()

    def resolve_dhtt_action(self, request: CookingRequest.Request, response: CookingRequest.Response):
        """
        Take an action from a dHTT CookingRequest and step the environment. Simple actions like move and interact
        return in one step. High-level actions like MoveTo may take several steps.
        :param request:
        :param response:
        """
        cooking_action = request.action

        if request.super_action != CookingRequest.Request.ACTION:
            msg = f'resolve_dhtt_action expects CookingRequest.Request.ACTION, got {request.super_action}'
            response.error_msg += msg
            raise RuntimeError(msg)

        if cooking_action.action_type < CookingAction.MOVE_TO:
            # if not a high-level command, we can assume the constants in CookingAction line up with the ActionScheme
            # actions, so we send them to the environment directly
            self._step_environment_from_resolved_action(cooking_action.action_type, cooking_action.player_name)
        elif cooking_action.action_type == CookingAction.MOVE_TO:
            # Now we parse and step through BaseAgent's MoveTo(). This runs as many steps as needed for the agent to
            # walk somewhere.
            if cooking_action.params == "":
                response.error_msg = f'Empty parameters'
                return
            try:
                loc = self._strip_loc(cooking_action.params)
            except ValueError as e:
                response.error_msg = f'Failed to parse location parameter: {e}'
                return

            is_moving = True
            while is_moving:
                agent_observation = self.observations[DEFAULT_PLAYER_NAME]
                self.my_agent.update_location(agent_observation)
                before_loc = self.my_agent.location

                action_chosen = self.my_agent.walk_to_location(loc, agent_observation)
                self._step_environment_from_resolved_action(action_chosen, cooking_action.player_name)

                # BaseAgent.MoveTo() lets the agent move into a countertop forever. Let this happen once to set the
                # orientation, then break the loop.
                agent_observation = self.observations[DEFAULT_PLAYER_NAME]
                self.my_agent.update_location(agent_observation)
                is_moving = before_loc != self.my_agent.location

    def get_observation_msg(self) -> CookingObservation:
        msg = CookingObservation()

        world_objects = self.env.unwrapped.world.world_objects
        world_agents = self.env.unwrapped.world.agents
        msg.objects = [self._object_to_object_msg(obj) for obj_list in world_objects.values() for obj in obj_list]

        closest = self._closest_of_each_type(world_objects)
        msg.closest_objects_types = list(closest.keys())
        msg.closest_objects_locations = [x[0] for x in closest.values()]
        msg.closest_objects_distances = [x[1] for x in closest.values()]

        msg.agents = [self._agent_to_agent_msg(obj) for obj in world_agents]

        msg.ticks = self.env.unwrapped.t

        msg.recipes_allowed = [self._recipe_to_recipe_msg(self.env.unwrapped.recipes[recipe_name]().root_node) for
                               recipe_name in self.env.unwrapped.recipe_names]
        msg.recipes_completed = [self._recipe_to_recipe_msg(node) for node in self.env.unwrapped.dhtt_completed_recipes]
        msg.recipes_duds = [self._recipe_to_recipe_msg(node) for node in self.env.unwrapped.dhtt_dud_recipes]

        return msg

    @staticmethod
    def _init_cooking_zoo():  # TODO refactor to be non-static
        """
        Helper to initialize the environment. Just moves this code from self.__init__()
        :return: tuple of dict terminations, dict truncations
        """
        num_agents = 1
        max_steps = 1_000_000
        render = True
        obs_spaces = ["symbolic"]
        action_scheme = "scheme1_twohand"
        meta_file = "dhtt_experiment"
        level = "dhtt_experiment"
        allowed_recipes = ["TomatoToastedBreadPlate", "CarrotBananaPlate", "ToastedBreadPlate", "AppleWatermelonPlate",
                           "ApplePlate", "WatermelonPlate"]
        end_condition_all_dishes = True
        agent_visualization = ["human"]
        reward_scheme = {"recipe_reward": 20, "max_time_penalty": -5, "recipe_penalty": -40, "recipe_node_reward": 0}
        env = parallel_env(level=level, meta_file=meta_file, num_agents=num_agents, max_steps=max_steps,
                           recipes=allowed_recipes,
                           agent_visualization=agent_visualization, obs_spaces=obs_spaces,
                           end_condition_all_dishes=end_condition_all_dishes, action_scheme=action_scheme,
                           render=render,
                           reward_scheme=reward_scheme, ignore_completed_recipes=True, agents_arms=[2])
        observations, info = env.reset()
        # env.render() # has to be in main thread
        terminations = {DEFAULT_PLAYER_NAME: False}
        truncations = {DEFAULT_PLAYER_NAME: False}

        return env, observations, info, terminations, truncations

    def _step_environment_from_resolved_action(self, action: int, player_name=DEFAULT_PLAYER_NAME):
        actions = {player_name: action}
        if not all(self.terminations.values()) or all(self.truncations.values()):
            observations, rewards, terminations, truncations, infos = self.env.step(actions)
            self.observations = observations
            self.last_reward = rewards[DEFAULT_PLAYER_NAME]
            # self.cumulative_reward += self.last_reward
            self.terminations = terminations
            self.truncations = truncations
            self.infos = infos

            # self.env.render() # has to be in main thread

    @staticmethod
    def _strip_loc(s: str):
        """Take CookingAction location param and turn into a tuple"""
        parts = s.strip().split(',')

        if len(parts) != 2:
            raise ValueError("Input must contain exactly two comma-separated values.")

        try:
            # ROS may send messages like '1.000000, 1.000000' which don't cast to an int, but do to a float
            num1 = int(float(parts[0].strip()))
            num2 = int(float(parts[1].strip()))
        except ValueError as e:
            raise ValueError("Both parts must be valid numbers.") from e

        return (num1, num2)

    def _walking_distance(self):
        pass  # TODO instead of euclidean distance used in BaseAgent.distance()

    def _closest_of_each_type(self, world_objects: Dict['str', List[cooking_zoo_Object]]) -> Dict[
        'str', Tuple[geometry_msgs.msg.Point, float]]:
        """
        :param world_objects:
        :return: Locations of and distances to the closest of each type of world object
        """
        to_ret = dict()
        for object_type_name, objects in world_objects.items():
            locs = [x.location for x in objects]
            self.my_agent.update_location(self.observations[DEFAULT_PLAYER_NAME])
            closest_loc: tuple = self.my_agent.closest(self.my_agent.location, locs,
                                                       self.observations[DEFAULT_PLAYER_NAME])
            if closest_loc is not None:
                # rosidl won't implicitly cast an int to geometry_msgs float
                closest_point = geometry_msgs.msg.Point(x=float(closest_loc[0]), y=float(closest_loc[1]))
                closest_loc_distance = self.my_agent.distance(self.my_agent.location, closest_loc)
                to_ret[object_type_name] = (closest_point, closest_loc_distance)
        return to_ret

    @staticmethod
    def _object_to_object_msg(obj: cooking_zoo_Object) -> CookingObject:
        to_ret = CookingObject()
        to_ret.world_id = obj.unique_id
        to_ret.moveable = obj.movable
        to_ret.walkable = obj.walkable
        to_ret.is_static = isinstance(obj, cooking_zoo_StaticObject)
        to_ret.object_type = type(obj).__name__
        to_ret.location.x = float(obj.location[0])
        to_ret.location.y = float(obj.location[1])
        # to_ret.physical_state = obj.physical_state
        to_ret.progress = to_ret.progress if hasattr(obj, 'current_progress') else -1

        if hasattr(obj, 'content'):
            to_ret.content_ids = [x.unique_id for x in obj.content]

        attrs = dir(obj)
        if 'fry_state' in attrs:
            to_ret.physical_state.append(obj.fry_state.value)
        if 'blend_state' in attrs:
            to_ret.physical_state.append(obj.blend_state.value)
        if 'chop_state' in attrs:
            to_ret.physical_state.append(obj.chop_state.value)
        if 'toast_state' in attrs:
            to_ret.physical_state.append(obj.toast_state.value)
        if 'boil_state' in attrs:
            to_ret.physical_state.append(obj.boil_state.value)

        return to_ret

    @classmethod
    def _agent_to_agent_msg(cls, agent: cooking_zoo_Agent) -> CookingAgent:
        to_ret = CookingAgent()
        to_ret.object_members = cls._object_to_object_msg(agent)
        to_ret.orientation = agent.orientation
        to_ret.holding = [cls._object_to_object_msg(x) for x in agent.holding if x is not None]

        return to_ret

    @classmethod
    def _recipe_to_recipe_msg(cls, recipe: cooking_zoo_RecipeNode) -> CookingRecipe:
        to_ret = CookingRecipe()
        to_ret.recipe_name = recipe.name

        # helper for adding recipe's required objects
        def recurse_into_nodes(node: cooking_zoo_RecipeNode):
            if node.contains is not None:
                for x in node.contains:
                    recurse_into_nodes(x)

            root_obj = node.root_type((0, 0))

            if node.conditions is not None:
                for cond in node.conditions:
                    try:
                        setattr(root_obj, cond[0], cond[1])
                    except AttributeError as ex:
                        print(ex)

            to_ret.required_objects.append(cls._object_to_object_msg(root_obj))

        recurse_into_nodes(recipe)

        return to_ret


def main():
    rclpy.init()

    my_ex = rclpy.executors.MultiThreadedExecutor()
    node = CookingNode(my_ex)

    my_ex.add_node(node)

    while rclpy.ok():
        my_ex.spin_once()
        node.cooking_environment.env.render()  # pygame/SDL 2.0 apparently needs to be in the main thread
        # TODO misses rendering move_to intermediate steps this is the simplest solution

    rclpy.shutdown()


if __name__ == '__main__':
    main()
