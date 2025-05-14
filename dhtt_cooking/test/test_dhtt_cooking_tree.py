#!/usr/bin/python

"""Boilerplate copied from dhtt tests"""

import pytest
import rclpy
import rclpy.node
import pathlib
import yaml

from threading import Lock

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, CookingRequest
from dhtt_msgs.msg import Subtree, Node, NodeStatus


class ServerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('test_node')
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
        assert self.modifysrv.wait_for_service(timeout_sec=5.0)

        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
        assert self.fetchsrv.wait_for_service(timeout_sec=5.0)

        self.controlsrv = self.create_client(ControlRequest, '/control_service')
        assert self.controlsrv.wait_for_service(timeout_sec=5.0)

        self.historysrv = self.create_client(HistoryRequest, '/history_service')
        assert self.historysrv.wait_for_service(timeout_sec=5.0)

        self.cooking_client = self.create_client(CookingRequest, '/Cooking_Server')
        assert self.historysrv.wait_for_service(timeout_sec=1.0)

        self.root_status_sub = self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)
        self.status_sub = self.create_subscription(Node, "/status", self.status_listener, 10)

        self.root_state = 0
        self.node_states = {}

    def root_status_listener(self, data):
        self.root_state = data.state

    def status_listener(self, data):
        # node_name = data.node_name.split('_')[0]

        self.node_states[data.node_name] = data

    def get_tree(self):
        fetch_rq = FetchRequest.Request()
        fetch_rq.return_full_subtree = True

        fetch_future = self.fetchsrv.call_async(fetch_rq)
        rclpy.spin_until_future_complete(self, fetch_future)

        fetch_rs = fetch_future.result()

        return fetch_rs

    def wait_for_node_in_state(self, node_name, state):
        while self.node_states[node_name].node_status.state != state and rclpy.ok():
            rclpy.spin_once(self)

    def reset_tree(self):
        fetch_rs = self.get_tree()

        if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:
            nodes_to_remove = [i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

            reset_rq = ControlRequest.Request()
            reset_rq.type = ControlRequest.Request.RESET

            reset_future = self.controlsrv.call_async(reset_rq)
            rclpy.spin_until_future_complete(self, reset_future)

            reset_rs = reset_future.result()

    def interrupt_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.STOP

        control_future = self.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self, control_future)

        control_rs = control_future.result()


class TestCookingZooTree:
    first = True
    node: ServerNode = None
    lock = Lock()
    rclpy.init()

    def initialize(self):

        if TestCookingZooTree.first:

            TestCookingZooTree.first = False

            TestCookingZooTree.node = ServerNode()

        self.ok = False

    def get_tree(self):
        fetch_rq = FetchRequest.Request()
        fetch_rq.return_full_subtree = True

        fetch_future = self.node.fetchsrv.call_async(fetch_rq)
        rclpy.spin_until_future_complete(self.node, fetch_future)

        fetch_rs = fetch_future.result()

        assert fetch_rs.success == True

        return fetch_rs

    def reset_tree(self):

        self.interrupt_tree()

        fetch_rs = self.get_tree()

        if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:
            nodes_to_remove = [i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

            reset_rq = ControlRequest.Request()
            reset_rq.type = ControlRequest.Request.RESET

            reset_future = self.node.controlsrv.call_async(reset_rq)
            rclpy.spin_until_future_complete(self.node, reset_future)

            reset_rs = reset_future.result()

            assert reset_rs.success == True

    def interrupt_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.STOP

        control_future = self.node.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self.node, control_future)

        control_rs = control_future.result()

    # asserts here will only work if this is the only way that nodes have been added to the tree
    def add_from_yaml(self, file_name, force, add_to="ROOT_0"):

        wd = pathlib.Path(__file__).parent.resolve()

        yaml_dict = None

        assert file_name.split('.')[-1] == 'yaml'

        with open(f'{wd}{file_name}', 'r') as file:
            yaml_dict = yaml.safe_load(file)

        assert yaml_dict != None

        modify_rq = ModifyRequest.Request()
        modify_rq.type = ModifyRequest.Request.ADD_FROM_FILE
        modify_rq.force = force

        modify_rq.to_modify.append(add_to)
        modify_rq.to_add = f'{wd}{file_name}'

        modify_future = self.node.modifysrv.call_async(modify_rq)
        rclpy.spin_until_future_complete(self.node, modify_future)

        modify_rs = modify_future.result()

        assert modify_rs.success == True

        fetch_rs = self.get_tree()

        node_names_orig = yaml_dict['NodeList']
        node_parents_orig = [yaml_dict['Nodes'][i]['parent'] for i in node_names_orig]

        node_names_from_server = [i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]
        parent_names_from_server = [i.parent_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

        # verify that all nodes were added
        for index, val in enumerate(node_names_orig):
            found = False;

            for index2, gt_val in enumerate(node_names_from_server):
                if val in gt_val:
                    found = True
                    break

            assert found

        # verify that each node has the correct parent
        for index, val in enumerate(node_parents_orig):
            found = False;

            for index2, gt_val in enumerate(parent_names_from_server):
                if val in gt_val:
                    found = True
                    break

            # assert found or (val == 'NONE' and parent_names_from_server[index] == add_to)

        goitr_names_from_server = [i.goitr_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

        # verify goitrs were added correctly
        for index, val in enumerate(yaml_dict['Nodes']):
            try:
                goitr_type_from_file = i['goitr_type']

                assert goitr_type_from_file == goitr_names_from_server[index]
            except:
                continue

        return modify_rs.added_nodes

    def load_expected_from_yaml(self, file_name):

        wd = pathlib.Path(__file__).parent.resolve()

        assert file_name.split('.')[-1] == 'yaml'

        with open(f'{wd}{file_name}', 'r') as file:
            yaml_dict = yaml.safe_load(file)

        assert yaml_dict != None
        assert len(yaml_dict["expected_order"]) > 0

        return yaml_dict["expected_order"]

    def start_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.START

        control_future = self.node.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self.node, control_future)

        control_rs = control_future.result()

        assert control_rs.success == True

        return control_rs

    def spin_sub(self):
        while self.ok:
            rclpy.spin_once(self.node)

            assert rclpy.ok()

    def wait_for_waiting(self):
        while TestCookingZooTree.node.root_state != NodeStatus.WAITING and rclpy.ok():
            rclpy.spin_once(TestCookingZooTree.node)

        assert rclpy.ok()

    def wait_for_finished_execution(self):

        # self.ok = True

        # spin_thread = Thread(target=self.spin_sub)
        # spin_thread.start()

        while TestCookingZooTree.node.root_state != NodeStatus.DONE and rclpy.ok():
            rclpy.spin_once(TestCookingZooTree.node)

        # self.ok = False
        # spin_thread.join()

        assert rclpy.ok()

    def get_history(self):
        history_rq = HistoryRequest.Request()

        history_future = self.node.historysrv.call_async(history_rq)
        rclpy.spin_until_future_complete(self.node, history_future)

        history_rs = history_future.result()

        return history_rs.node_history

    def compare_history_to_expected(self, history, expected):
        assert len(history) == len(expected)

        for index, val in enumerate(expected):
            assert val == history[index].split('_')[0]

    def order_from_file_test(self, file_name):

        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()

            self.wait_for_waiting()

            expected = self.load_expected_from_yaml(file_name)

            self.add_from_yaml(file_name, force=True)

            self.start_tree()

            self.wait_for_finished_execution()

            history = self.get_history()

            self.compare_history_to_expected(history, expected)
            self.reset_tree()

    def reset_level(self):
        request = CookingRequest.Request()
        request.super_action = CookingRequest.Request.START

        future = self.node.cooking_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        res = future.result()

        assert res
        return res

    def test_known_good(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()
            self.add_from_yaml("/test_descriptions/simple_and.yaml", force=True)
            self.start_tree()
            self.wait_for_finished_execution()
            history = self.get_history()
            self.reset_tree()

    def test_single_node(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()
            self.add_from_yaml("/test_descriptions/test_cooking_singlenode.yaml", force=True)
            self.start_tree()
            self.wait_for_finished_execution()
            history = self.get_history()
            self.reset_tree()

    def test_multiple_move(self):
        """
        If this fails, consider that the agent may have spawned somewhere that messed up the expected order.
        Or change this to not use expected_order
        """
        self.order_from_file_test("/test_descriptions/test_cooking_multiplemove.yaml")

    def test_multiple_moveobject(self):
        """
        If this fails, consider that the agent may have spawned somewhere that messed up the expected order.
        Or change this to not use expected_order
        """
        self.order_from_file_test("/test_descriptions/test_cooking_multiplemoveobject.yaml")

    def test_execute(self):
        self.order_from_file_test("/test_descriptions/test_cooking_execute.yaml")

    def test_pick(self):
        self.order_from_file_test("/test_descriptions/test_cooking_executepick.yaml")

    def test_pickplace(self):
        self.order_from_file_test("/test_descriptions/test_cooking_pickplace.yaml")

    @pytest.mark.skip("TODO Failing")
    def test_interactspecial(self):
        self.order_from_file_test("/test_descriptions/test_cooking_interactspecial.yaml")

    @pytest.mark.skip(reason="TODO Failing")
    def test_tomatolettucesalad(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()
            self.add_from_yaml("/test_descriptions/test_cooking_recipetomatolettucesalad.yaml", force=True)
            self.start_tree()
            self.wait_for_finished_execution()
            history = self.get_history()
            self.reset_tree()

    @pytest.mark.skip(reason="TODO Failing")
    def test_two_orders_simple_And(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()

            req = ModifyRequest.Request()
            req.type = ModifyRequest.Request.ADD
            req.to_modify.append('ROOT_0')
            req.add_node = Node()
            req.add_node.type = Node.AND
            req.add_node.node_name = 'AllOrdersAnd'
            req.add_node.plugin_name = 'dhtt_plugins::AndBehavior'
            fut = self.node.modifysrv.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut)
            true_name = fut.result().added_nodes[0]

            self.add_from_yaml("/test_descriptions/test_cooking_two_orders_simple.yaml", force=True, add_to=true_name)
            self.add_from_yaml("/test_descriptions/test_cooking_two_orders_simple.yaml", force=True, add_to=true_name)
            self.start_tree()
            self.wait_for_finished_execution()
            self.reset_tree()

    @pytest.mark.skip(reason="TODO Failing")
    def test_two_orders_simple_Then(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()

            req = ModifyRequest.Request()
            req.type = ModifyRequest.Request.ADD
            req.to_modify.append('ROOT_0')
            req.add_node = Node()
            req.add_node.type = Node.THEN
            req.add_node.node_name = 'AllOrdersThen'
            req.add_node.plugin_name = 'dhtt_plugins::ThenBehavior'
            fut = self.node.modifysrv.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut)
            true_name = fut.result().added_nodes[0]

            self.add_from_yaml("/test_descriptions/test_cooking_two_orders_simple.yaml", force=True, add_to=true_name)
            self.add_from_yaml("/test_descriptions/test_cooking_two_orders_simple.yaml", force=True, add_to=true_name)
            self.start_tree()
            self.wait_for_finished_execution()
            self.reset_tree()

    @pytest.mark.skip(reason="TODO Failing")
    def test_mark(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()

            req = ModifyRequest.Request()
            req.type = ModifyRequest.Request.ADD
            req.to_modify.append('ROOT_0')
            req.add_node = Node()
            req.add_node.type = Node.AND
            req.add_node.node_name = 'AllOrdersAnd'
            req.add_node.plugin_name = 'dhtt_plugins::AndBehavior'
            fut = self.node.modifysrv.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut)
            true_name = fut.result().added_nodes[0]

            self.add_from_yaml("/test_descriptions/test_cooking_pickplace_mark.yaml", force=True,
                               add_to=true_name)
            self.add_from_yaml("/test_descriptions/test_cooking_pickplace_mark.yaml", force=True,
                               add_to=true_name)
            self.start_tree()
            self.wait_for_finished_execution()
            self.reset_tree()

    ### BEGIN EXPERIMENT TESTS ###

    @pytest.mark.skip(reason="TODO Failing")
    def test_lettucesalad_experiment(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()

            req = ModifyRequest.Request()
            req.type = ModifyRequest.Request.ADD
            req.to_modify.append('ROOT_0')
            req.add_node = Node()
            req.add_node.type = Node.AND
            req.add_node.node_name = 'AllOrdersAnd'
            req.add_node.plugin_name = 'dhtt_plugins::AndBehavior'
            fut = self.node.modifysrv.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut)
            true_name = fut.result().added_nodes[0]

            self.add_from_yaml("/experiment_descriptions/recipe_lettucesalad.yaml", force=True,
                               add_to=true_name)
            self.add_from_yaml("/experiment_descriptions/recipe_lettucesalad.yaml", force=True,
                               add_to=true_name)
            self.start_tree()
            self.wait_for_finished_execution()
            # self.reset_tree()

    @pytest.mark.skip(reason="TODO Failing")
    def test_tomatotoast_experiment(self):
        with TestCookingZooTree.lock:
            self.initialize()
            self.reset_level()
            self.reset_tree()
            self.wait_for_waiting()

            req = ModifyRequest.Request()
            req.type = ModifyRequest.Request.ADD
            req.to_modify.append('ROOT_0')
            req.add_node = Node()
            req.add_node.type = Node.THEN
            req.add_node.node_name = 'AllOrdersThen'
            req.add_node.plugin_name = 'dhtt_plugins::ThenBehavior'
            fut = self.node.modifysrv.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut)
            true_name = fut.result().added_nodes[0]

            nodes = self.add_from_yaml("/experiment_descriptions/recipe_tomatotoast.yaml", force=True, add_to=true_name)
            self.add_from_yaml("/experiment_descriptions/recipe_tomatotoast.yaml", force=True, add_to=true_name)
            self.start_tree()

            # TestCookingZooTree.node.wait_for_node_in_state("TomatoToastBreadChoppedBreadExists_67", NodeStatus.WORKING)
            #
            # TestCookingZooTree.node.interrupt_tree()

            self.wait_for_finished_execution()
            self.reset_tree()

    # TODO negative tests
