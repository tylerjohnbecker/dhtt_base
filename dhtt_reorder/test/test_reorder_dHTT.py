import pytest
from dhtt_plot.build_nutree import dHTTHelpers
from dhtt_reorder.dHTT_reorder import HTT

import rclpy
import rclpy.logging
import rclpy.node
import pathlib
import yaml
import re

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, NutreeJsonRequest
from dhtt_msgs.msg import Subtree, Node, NodeStatus
from dhtt_msgs.msg import Subtree as dHTTSubtree, Node as dHTTNode

from nutree import Tree as nutreeTree, Node as nutreeNode, IterMethod
from io import StringIO


class ServerNode (rclpy.node.Node):

    def __init__(self):
        super().__init__('test_node')
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
        assert self.modifysrv.wait_for_service(timeout_sec=1.0)

        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
        assert self.fetchsrv.wait_for_service(timeout_sec=1.0)

        self.controlsrv = self.create_client(
            ControlRequest, '/control_service')
        assert self.controlsrv.wait_for_service(timeout_sec=1.0)

        self.historysrv = self.create_client(
            HistoryRequest, '/history_service')
        assert self.historysrv.wait_for_service(timeout_sec=1.0)

        self.root_status_sub = self.create_subscription(
            NodeStatus, '/root_status', self.root_status_listener, 10)
        self.status_sub = self.create_subscription(
            Node, "/status", self.status_listener, 10)

        self.nutreeServerClient = self.create_client(
            NutreeJsonRequest, '/nutree_json_service')
        assert self.nutreeServerClient.wait_for_service(
            timeout_sec=1.0), "Did you forget?: ros2 run dhtt_plot build_nutree_server"

        self.root_state = 0
        self.node_states = {}

    def root_status_listener(self, data):
        self.root_state = data.state

    def status_listener(self, data):
        node_name = data.node_name.split('_')[0]

        self.node_states[node_name] = data


class TestdHTTReorder:
    rclpy.init()
    node = ServerNode()

    def get_tree(self):
        fetch_rq = FetchRequest.Request()
        fetch_rq.return_full_subtree = True

        fetch_future = self.node.fetchsrv.call_async(fetch_rq)
        rclpy.spin_until_future_complete(self.node, fetch_future)

        fetch_rs = fetch_future.result()

        assert fetch_rs.success == True

        return fetch_rs

    def reset_tree(self):
        self.node.node_states = {}

        fetch_rs = self.get_tree()

        if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:

            nodes_to_remove = [
                i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

            reset_rq = ControlRequest.Request()
            reset_rq.type = ControlRequest.Request.RESET

            reset_future = self.node.controlsrv.call_async(
                reset_rq)
            rclpy.spin_until_future_complete(
                self.node, reset_future)

            reset_rs = reset_future.result()

            assert reset_rs.success == True

    def createExampleTree(self, file_name, add_to=dHTTHelpers.ROOTNAME):
        # copied from add_from_yaml in dhtt tests
        yaml_dict = None
        print(file_name)
        assert file_name.split('.')[-1] == 'yaml'
        with open(file_name, 'r') as file:
            yaml_dict = yaml.safe_load(file)
        assert yaml_dict != None

        modifyRQ = ModifyRequest.Request()
        modifyRQ.type = ModifyRequest.Request.ADD_FROM_FILE

        modifyRQ.to_modify.append(add_to)
        modifyRQ.to_add = file_name

        modifyFuture = self.node.modifysrv.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, modifyFuture)

        modifyRS: ModifyRequest.Response = modifyFuture.result()

        assert modifyRS.success == True

    def test_exampleTree(self):
        self.reset_tree()
        client = HTT(withdHTT=True)

        self.createExampleTree(
            f'{pathlib.Path(__file__).parent.resolve()}/yaml/exampleABCD.yaml')

        rs = client.setTreeFromdHTT()
        self.reset_tree()
        assert rs

        client = HTT(withdHTT=False)
        rs = client.setTreeFromdHTT()
        assert rs == False

    def test_helpers(self):
        self.reset_tree()
        htt = HTT(withdHTT=True)
        self.createExampleTree(
            f'{pathlib.Path(__file__).parent.resolve()}/yaml/complex_tree.yaml')
        htt.setTreeFromdHTT()
        self.reset_tree()

        andNode = next(x for x in htt.tree if x.data.type == dHTTNode.AND)
        thenNode = next(x for x in htt.tree if x.data.type == dHTTNode.THEN)
        behaviorNode = next(
            x for x in htt.tree if x.data.type == dHTTNode.BEHAVIOR)

        assert htt.isTaskNode(andNode)
        assert htt.isTaskNode(thenNode)
        assert htt.isTaskNode(behaviorNode) == False

        assert htt.isThenNode(andNode) == False
        assert htt.isThenNode(thenNode)
        assert htt.isThenNode(behaviorNode) == False

        assert htt.isAndNode(andNode)
        assert htt.isAndNode(thenNode) == False
        assert htt.isAndNode(behaviorNode) == False

        assert htt.isBehaviorNode(andNode) == False
        assert htt.isBehaviorNode(thenNode) == False
        assert htt.isBehaviorNode(behaviorNode)

    def test_reorderABCD(self):
        self.reset_tree()

        def findNode(targetName):
            r = re.compile(f'^{targetName}_[0-9]+$')
            return next(x for x in htt.tree if r.match(x.data.node_name))

        # ab, bc, ca -> AND,d,THEN,c,THEN,a,b
        htt = HTT(withdHTT=True)
        self.createExampleTree(
            f'{pathlib.Path(__file__).parent.resolve()}/yaml/exampleABCD.yaml')
        htt.setTreeFromdHTT()

        before = [findNode('A')]
        after = [findNode('B')]
        htt.reorder(before, after, debug=True)

        before = [findNode('B')]
        after = [findNode('C')]
        htt.reorder(before, after, debug=True)

        before = [findNode('C')]
        after = [findNode('A')]
        htt.reorder(before, after, debug=True)

        htt.diffMergedHTT(htt.tree)

        assert [(x.node_name, x.parent_name) for x in self.get_tree().found_subtrees[0].tree_nodes] == [('ROOT_0', 'NONE'), ('TopAnd_1_6', 'ROOT_0'),
                                                                                                        ('D_5', 'TopAnd_1_6'), ('THEN_7', 'TopAnd_1_6'), ('C_4', 'THEN_7'), ('THEN_8', 'THEN_7'), ('A_2', 'THEN_8'), ('B_3', 'THEN_8')]

        self.reset_tree()
        assert True

    def test_add_multipletoroot(self):
        self.reset_tree()
        modifyRQ = ModifyRequest.Request()
        modifyRQ.type = ModifyRequest.Request.ADD

        modifyRQ.to_modify.append('ROOT_0')

        modifyRQ.add_node = Node()
        modifyRQ.add_node.type = Node.BEHAVIOR
        modifyRQ.add_node.node_name = 'Node'
        modifyRQ.add_node.plugin_name = 'dhtt_plugins::TestBehavior'

        future = self.node.modifysrv.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, future)

        future = self.node.modifysrv.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, future)

        htt = HTT(withdHTT=True)
        htt.setTreeFromdHTT()
        assert [x.data.node_name for x in htt.tree] == ['Node_1', 'Node_2']

        self.reset_tree()

        # resets

    def test_reorder_complexTree(self):
        pass
