import re

import rclpy
import rclpy.logging
import rclpy.node
import geometry_msgs.msg

from dhtt_msgs.msg import CookingAction, CookingAgent, CookingObservation, CookingObject, CookingRecipe
from dhtt_msgs.srv import CookingRequest, CookingRequest_Request, CookingRequest_Response

from cooking_zoo.cooking_agents.base_agent import BaseAgent
from cooking_zoo.environment.cooking_env import CookingEnvironment as CookingZooEnvironment, parallel_env
from cooking_zoo.cooking_world.abstract_classes import Object as cooking_zoo_Object
from cooking_zoo.cooking_world.world_objects import Agent as cooking_zoo_Agent
from cooking_zoo.cooking_book.recipe import RecipeNode as cooking_zoo_RecipeNode

DEFAULT_PLAYER_NAME = CookingAction.DEFAULT_PLAYER_NAME  # usually player_0 this is from cooking_zoo cooking_env.py
DEFAULT_AGENT_ID = CookingAction.DEFAULT_AGENT_ID  # usually agent-1. This is the name given to the actual agent object


class CookingNode(rclpy.node.Node):
    """
    Class for handling requests to the cooking_zoo environment

    Starts up ROS **services** and callbacks and keeps an internal instance of the cooking_zoo environment.

    For tight control, runs as a synchronous loop: Upon receiving a service request, steps the environment as many
    times as needed before returning. For some actions, this may be one environment tick, for others it may be several.
    Without a service request, the environment will not tick. To 'busy wait' in the environment, send a series of NO_OP
    actions.
    """

    def __init__(self):
        super().__init__('cooking_zoo')

        self.cooking_environment = CookingEnvironment()
        self.cooking_server = self.create_service(CookingRequest, 'Cooking_Server', self.cooking_request_callback)
        self.cooking_observation_publisher = self.create_publisher(CookingObservation, 'Cooking_Observations', 10)

        self.get_logger().debug('Initialized' + self.get_name())

        self.actions_taken: list[CookingAction] = []

        if self.cooking_environment:
            obs = self._prepare_observation_msg()
            self.cooking_observation_publisher.publish(obs)
            self.get_logger().info('Published initial observation')
            self.get_logger().debug(f'Initial obs: {obs}')

    def cooking_request_callback(self, request: CookingRequest_Request,
                                 response: CookingRequest_Response) -> CookingRequest_Response:
        to_ret = ""
        if request.super_action == CookingRequest.Request.START:
            self.get_logger().info('Got start request')
            self.cooking_environment.start()
        elif request.super_action == CookingRequest.Request.STOP:
            self.get_logger().info('Got stop request')
            self.cooking_environment.stop()
        elif request.super_action == CookingRequest.Request.RESET:
            self.get_logger().info('Got reset request')
            self.cooking_environment.reset()
        elif request.super_action == CookingRequest.Request.ACTION:
            self.get_logger().info('Got action request')
            if request.action.player_name == "":
                self.get_logger().error('Bad action request: No player name')
                to_ret = "Bad action request: No player name"
            else:
                self.get_logger().debug(
                    f'Resolving action request with: player {request.action.player_name}, action_type {request.action.action_type}, params {request.action.params}')
                self.cooking_environment.resolve_dhtt_action(request, response)

                if response.error_msg == "":
                    self.actions_taken.append(request.action)
        else:
            to_ret = "No valid super_action"

        if self.cooking_environment:
            obs = self._prepare_observation_msg()
            self.cooking_observation_publisher.publish(obs)
            self.get_logger().debug(f'Published observation, header: {obs.head}')

        response.error_msg += to_ret
        response.success = response.error_msg == ""

        return response

    def _prepare_observation_msg(self) -> CookingObservation:
        obs = self.cooking_environment.get_observation_msg()
        obs.actions = self.actions_taken
        return obs


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

    def resolve_dhtt_action(self, request: CookingRequest_Request, response: CookingRequest_Response):
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
        max_steps = 400
        render = True
        obs_spaces = ["symbolic"]
        action_scheme = "scheme1_twohand"
        meta_file = "example_absorbing_toaster"
        level = "switch_test_absorbing_toaster"
        allowed_recipes = ["TomatoLettucePlate", "MashedCarrotPlate", "ToastedBreadPlate"]
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
        env.render()
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

            self.env.render()

    @staticmethod
    def _strip_loc(s: str):
        """Take CookingAction location param and turn into a tuple"""
        parts = s.strip().split(',')

        if len(parts) != 2:
            raise ValueError("Input must contain exactly two comma-separated values.")

        try:
            num1 = int(parts[0].strip())
            num2 = int(parts[1].strip())
        except ValueError as e:
            raise ValueError("Both parts must be valid numbers.") from e

        return (num1, num2)

    def _walking_distance(self):
        pass  # TODO instead of euclidean distance used in BaseAgent.distance()

    def _closest_of_each_type(self, world_objects: dict['str', list[cooking_zoo_Object]]) -> dict[
        'str', tuple[geometry_msgs.msg.Point, float]]:
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
        to_ret.object_type = type(obj).__name__
        to_ret.location.x = float(obj.location[0])
        to_ret.location.y = float(obj.location[1])
        # to_ret.physical_state = obj.physical_state
        to_ret.progress = to_ret.progress if hasattr(obj, 'current_progress') else -1

        if hasattr(obj, 'content'):
            to_ret.content_ids = [x.unique_id for x in obj.content]

        attrs = dir(obj)
        if 'chop_state' in attrs:
            to_ret.physical_state.append(obj.chop_state.value)
        if 'blend_state' in attrs:
            to_ret.physical_state.append(obj.blend_state.value)
        if 'toast_state' in attrs:
            to_ret.physical_state.append(obj.toast_state.value)

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

    node = CookingNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
