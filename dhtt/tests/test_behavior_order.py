#!/usr/bin/python

import pytest
import rclpy
import rclpy.node
import pathlib
import yaml
import copy
import contextlib
import os

import filelock

from functools import partial
from threading import Lock

from std_msgs.msg import String

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, GoitrRequest
from dhtt_msgs.msg import Subtree, Node, NodeStatus, Result  

@pytest.fixture(scope='session')
def lock(tmp_path_factory):
    base_temp = tmp_path_factory.getbasetemp()
    lock_file = base_temp.parent / 'serial.lock'
    yield filelock.FileLock(lock_file=str(lock_file))
    with contextlib.suppress(OSError):
        os.remove(path=lock_file)


@pytest.fixture()
def serial(lock):
    with lock.acquire(poll_interval=0.1):
        yield

class ServerNode (rclpy.node.Node):

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

		self.status_sub = self.create_subscription(NodeStatus, '/root_status', self.status_listener, 10)

		self.root_state = 0

	def status_listener(self, data):
		self.root_state = data.state

class TestServerBehaviorOrder:

	first = True
	node = None
	lock = Lock()
	
	def initialize(self):

		if TestServerBehaviorOrder.first:
			rclpy.init()

			TestServerBehaviorOrder.first = False

			TestServerBehaviorOrder.node = ServerNode()

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

		fetch_rs = self.get_tree()

		if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:

			nodes_to_remove = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

			reset_rq = ControlRequest.Request()
			reset_rq.type = ControlRequest.Request.RESET

			reset_future = self.node.controlsrv.call_async(reset_rq)
			rclpy.spin_until_future_complete(self.node, reset_future)

			reset_rs = reset_future.result()

			assert reset_rs.success == True

	# asserts here will only work if this is the only way that nodes have been added to the tree
	def add_from_yaml(self, file_name, force=True, add_to="ROOT_0"):

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
		node_parents_orig = [ yaml_dict['Nodes'][i]['parent'] for i in node_names_orig ]

		node_names_from_server = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]
		parent_names_from_server = [ i.parent_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

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

			assert found or (val == 'NONE' and parent_names_from_server[index] == add_to)

		goitr_names_from_server = [ i.goitr_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

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
		while TestServerBehaviorOrder.node.root_state != NodeStatus.WAITING and rclpy.ok():
			rclpy.spin_once(TestServerBehaviorOrder.node)
			
		assert rclpy.ok()

	def wait_for_finished_execution(self):

		# self.ok = True

		# spin_thread = Thread(target=self.spin_sub)
		# spin_thread.start()

		while TestServerBehaviorOrder.node.root_state != NodeStatus.DONE and rclpy.ok():
			rclpy.spin_once(TestServerBehaviorOrder.node)

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

		with TestServerBehaviorOrder.lock:
			self.initialize()
			self.reset_tree()

			self.wait_for_waiting()

			expected = self.load_expected_from_yaml(file_name)

			self.add_from_yaml(file_name)

			self.start_tree()

			self.wait_for_finished_execution()

			history = self.get_history()

			self.compare_history_to_expected(history, expected)

	def test_simple_and(self, serial):
		self.order_from_file_test("/test_descriptions/simple_and.yaml")

	def test_simple_or(self, serial):
		self.order_from_file_test("/test_descriptions/simple_or.yaml")

	def test_simple_then(self, serial):
		self.order_from_file_test("/test_descriptions/simple_then.yaml")

	def test_then_parent_or(self, serial):
		self.order_from_file_test("/test_descriptions/then_parent_or.yaml")

	def test_then_parent_and(self, serial):
		self.order_from_file_test("/test_descriptions/then_parent_and.yaml")

	def test_and_parent_or(self, serial):
		self.order_from_file_test("/test_descriptions/and_parent_or.yaml")

	def test_and_parent_then(self, serial):
		self.order_from_file_test("/test_descriptions/and_parent_then.yaml")

	def test_or_parent_then(self, serial):
		self.order_from_file_test("/test_descriptions/or_parent_then.yaml")

	def test_or_parent_and(self, serial):
		self.order_from_file_test("/test_descriptions/or_parent_and.yaml")

	def test_complex_tree(self, serial):
		self.order_from_file_test("/test_descriptions/complex_tree.yaml")

