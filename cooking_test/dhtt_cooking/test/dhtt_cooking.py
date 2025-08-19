from array import array
from typing import Tuple, List

import geometry_msgs.msg
import pytest
from cooking_zoo.cooking_world.constants import ChopFoodStates, ToasterFoodStates

from dhtt_cooking_msgs.msg import CookingAction, CookingObservation, CookingObject, CookingRecipe
from dhtt_cooking_msgs.srv import CookingRequest

import rclpy
import rclpy.logging
import rclpy.node

from dhtt_cooking.dhtt_cooking import CookingEnvironment
from dhtt_cooking.dhtt_cooking import DEFAULT_PLAYER_NAME, DEFAULT_AGENT_ID

from cooking_zoo.cooking_world.world_objects import Lettuce, Agent, Counter, Floor, Tomato, Bread
from cooking_zoo.cooking_book.recipe_drawer import RECIPES


class CookingClient(rclpy.node.Node):
    def __init__(self):
        super().__init__('CookingClient_test')
        self.cooking_request_srv = self.create_client(CookingRequest, 'Cooking_Server')
        assert self.cooking_request_srv.wait_for_service(
            timeout_sec=1.0), "Did you forget? ros2 run dhtt_cooking dhtt_cooking"

        self.cooking_observation_subscriber = self.create_subscription(CookingObservation, 'Cooking_Observations',
                                                                       self.obs_callback, 10)

        self.received_messages: List[CookingObservation] = []

    def obs_callback(self, msg: CookingObservation):
        print("Got topic message")
        self.received_messages.append(msg)


def make_request(super_action: int = 0, action_type: int = 0, params: str = '',
                 player_name: str = DEFAULT_PLAYER_NAME) -> Tuple[CookingRequest.Request, CookingRequest.Response]:
    request = CookingRequest.Request()
    request.super_action = super_action
    request.action.action_type = action_type
    request.action.params = params
    request.action.player_name = player_name
    return request, CookingRequest.Response()


class TestCookingEnvironment:
    @staticmethod
    def update_agent_loc(cenv: CookingEnvironment, player_name: str) -> Tuple[int, int]:
        cenv.my_agent.update_location(cenv.observations[player_name])
        return cenv.my_agent.location

    def test_init(self):
        assert CookingEnvironment()

    def test_start(self):
        cenv = CookingEnvironment()
        cenv.start()

    def test_stop(self):
        cenv = CookingEnvironment()
        cenv.start()
        cenv.stop()

    def test_reset(self):
        cenv = CookingEnvironment()
        cenv.reset()

    def test_resolve_dhtt_action(self):
        cenv = CookingEnvironment()
        req, res = make_request(CookingRequest.Request.START)
        with pytest.raises(RuntimeError):
            cenv.resolve_dhtt_action(req, res)
        assert len(res.error_msg) > 0
        # assert res.success == False # caught and set in server callback

        for i in range(CookingAction.NO_OP, CookingAction.EXECUTE_ACTION_ARM2 + 1):
            # magic numbers, I know EXECUTE_ACTION_ARM2 is the highest low-level action.
            # Also fun, cooking_zoo doesn't complain if you give it nonsense actions, it effectively just nops.
            req, res = make_request(CookingRequest.Request.ACTION, i)
            cenv.resolve_dhtt_action(req, res)

        could_move = False
        for movement in cenv.env.unwrapped.action_scheme_class.WALK_ACTIONS:
            req, res = make_request(CookingRequest.Request.ACTION, movement)

            loc_before = self.update_agent_loc(cenv, req.action.player_name)

            cenv.resolve_dhtt_action(req, res)

            # sometimes the agent is against a wall, so they can't move in that direction. Make sure at least one
            # motion worked
            could_move |= self.update_agent_loc(cenv, req.action.player_name) != loc_before
        assert could_move

    def test_resolve_dhtt_move_to(self):
        cenv = CookingEnvironment()

        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "1,1")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (1, 1)

        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "5,5")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (5, 5)

        # unreachable location, but the agent should face in that direction
        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "6,5")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (5, 5)
        assert cenv.my_agent.agent.orientation == CookingAction.WALK_RIGHT

        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "7,5")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (5, 5)
        assert cenv.my_agent.agent.orientation == CookingAction.WALK_RIGHT

        # this one is determined to be unreachable, so the agent doesn't move
        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "5,7")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (5, 5)
        assert cenv.my_agent.agent.orientation == CookingAction.WALK_RIGHT  # RIGHT not DOWN

        # this one is right on the edge (countertop) which BaseAgent.walk_to_location considers reachable and the agent
        # changes orientation at least
        req, res = make_request(CookingRequest.Request.ACTION, CookingAction.MOVE_TO, "5,6")
        cenv.resolve_dhtt_action(req, res)
        assert self.update_agent_loc(cenv, req.action.player_name) == (5, 5)
        assert cenv.my_agent.agent.orientation == CookingAction.WALK_DOWN

    def test_object_to_object_msg(self):
        cenv = CookingEnvironment()
        for obj in {Lettuce((0, 0)), Agent((0, 0), '', 'foo', 1), Counter((0, 0)), Floor((0, 0))}:
            msg = cenv._object_to_object_msg(obj)
            assert msg
            assert isinstance(msg.physical_state, List)
            assert isinstance(msg.location.x, float)
            assert isinstance(msg.location.y, float)
            assert isinstance(msg.location.z, float)

        assert "Fresh" in cenv._object_to_object_msg(Lettuce((0, 0))).physical_state

        bread = Bread((0, 0))
        bread.chop_state = ChopFoodStates.CHOPPED
        bread.toast_state = ToasterFoodStates.TOASTED
        msg = cenv._object_to_object_msg(bread)
        assert "Chopped" in msg.physical_state and "Toasted" in msg.physical_state

    def test_agent_to_agent_msg(self):
        cenv = CookingEnvironment()
        agent = Agent((0, 0), '', 'foo', 3)
        lettuce = Lettuce((0, 0))
        tomato = Tomato((0, 0))
        # third arm is None

        agent.grab(lettuce)
        agent.grab(tomato)
        assert lettuce in agent.holding and tomato in agent.holding

        msg = cenv._agent_to_agent_msg(agent)
        assert msg
        assert any([x.world_id == lettuce.unique_id for x in msg.holding]) and lettuce.unique_id != tomato.unique_id
        assert any([x.world_id == tomato.unique_id for x in msg.holding])
        assert cenv._object_to_object_msg(tomato) in msg.holding

    def test_recipe_to_recipe_msg(self):
        cenv = CookingEnvironment()
        for key, value in RECIPES.items():
            msg = cenv._recipe_to_recipe_msg(value().root_node)
            for x in msg.required_objects:
                assert isinstance(x, CookingObject)
            assert msg

        msg = cenv._recipe_to_recipe_msg(RECIPES['ToastedBreadPlate']().root_node)
        for x in [obj for obj in msg.required_objects if obj.object_type == 'Bread']:
            assert 'Chopped' in x.physical_state
            assert 'Toasted' in x.physical_state

    def test_get_observation_msg(self):
        cenv = CookingEnvironment()
        assert cenv._closest_of_each_type(cenv.env.unwrapped.world.world_objects)

        msg = cenv.get_observation_msg()

        assert isinstance(msg.objects, List)
        assert all(isinstance(obj, CookingObject) for obj in msg.objects)

        assert isinstance(msg.closest_objects_types, List)
        assert all(isinstance(obj, str) for obj in msg.closest_objects_types)

        assert isinstance(msg.closest_objects_locations, List)
        assert all(isinstance(obj, geometry_msgs.msg.Point) for obj in msg.closest_objects_locations)
        assert all(isinstance(point.x, float) for point in msg.closest_objects_locations)
        assert all(isinstance(point.y, float) for point in msg.closest_objects_locations)
        assert all(isinstance(point.z, float) for point in msg.closest_objects_locations)

        assert isinstance(msg.closest_objects_distances, List) or isinstance(msg.closest_objects_distances, array)
        assert all(isinstance(obj, float) for obj in msg.closest_objects_distances)


class TestCookingClient:
    rclpy.init()
    node = CookingClient()

    def test_client(self):
        def msg_success(msg):
            future = self.node.cooking_request_srv.call_async(msg)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1)
            return future.result().success

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.START
        assert msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.RESET
        assert msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.STOP
        assert msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.STOP
        assert msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.START
        assert msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = 88
        assert not msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        assert not msg_success(msg)

        obs_before = self.node.received_messages[-1]
        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        msg.action.action_type = CookingAction.WALK_RIGHT
        msg.action.player_name = DEFAULT_PLAYER_NAME
        assert msg_success(msg)
        assert obs_before.ticks == self.node.received_messages[-1].ticks - 1

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        msg.action.action_type = CookingAction.MOVE_TO
        msg.action.player_name = DEFAULT_PLAYER_NAME
        assert not msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        msg.action.action_type = CookingAction.MOVE_TO
        msg.action.player_name = DEFAULT_PLAYER_NAME
        msg.action.params = 'sadf'
        assert not msg_success(msg)

        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        msg.action.action_type = CookingAction.MOVE_TO
        msg.action.player_name = DEFAULT_PLAYER_NAME
        msg.action.params = '1,1'
        assert msg_success(msg)

        obs_before = self.node.received_messages[-1]
        msg = CookingRequest.Request()
        msg.super_action = CookingRequest.Request.ACTION
        msg.action.action_type = CookingAction.MOVE_TO
        msg.action.player_name = DEFAULT_PLAYER_NAME
        msg.action.params = '   5  ,  5  '
        assert msg_success(msg)
        assert obs_before is self.node.received_messages[-2]
        assert obs_before.ticks < self.node.received_messages[-1].ticks - 1
