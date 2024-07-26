#!/usr/bin/python3

import pytest
import rclpy
import rclpy.node
import pathlib
import yaml
import copy

from threading import Lock

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, GoitrRequest
from dhtt_msgs.msg import Subtree, Node, NodeStatus 

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

		self.root_status_sub = self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)
		self.status_sub = self.create_subscription(Node, "/status", self.status_listener, 10)

		self.root_state = 0
		self.node_states = {}
		self.updated_states = {}

	# make a client to the parent_topic of the goitr, and then return that client
	def create_goitr_parent_client(self, node_name):
		goitr_parent_client = self.create_client(GoitrRequest, node_name + "/goitr_parent_interface")

		max_wait_time = 10
		cur_wait_time = 0

		while not goitr_parent_client.wait_for_service(timeout_sec=1.0) and cur_wait_time < max_wait_time:
			cur_wait_time += 1

		assert cur_wait_time < max_wait_time

		return goitr_parent_client

	def root_status_listener(self, data):
		self.root_state = data.state

	def status_listener(self, data):
		node_name = data.node_name.split('_')[0]

		self.node_states[node_name] = data
		self.updated_states[node_name] = True

class TestGoitrSimple:

	first = True
	node = None
	lock = Lock()
	
	def initialize(self):

		if TestGoitrSimple.first:
			rclpy.init()

			TestGoitrSimple.first = False

			TestGoitrSimple.node = ServerNode()

		self.ok = False

	def get_tree(self):
		fetch_rq = FetchRequest.Request()
		fetch_rq.return_full_subtree = True

		fetch_future = TestGoitrSimple.node.fetchsrv.call_async(fetch_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, fetch_future)

		fetch_rs = fetch_future.result()

		assert fetch_rs.success == True

		return fetch_rs

	def reset_tree(self):

		TestGoitrSimple.node.node_states = {}

		fetch_rs = self.get_tree()

		if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:

			nodes_to_remove = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

			reset_rq = ControlRequest.Request()
			reset_rq.type = ControlRequest.Request.RESET

			reset_future = TestGoitrSimple.node.controlsrv.call_async(reset_rq)
			rclpy.spin_until_future_complete(TestGoitrSimple.node, reset_future)

			reset_rs = reset_future.result()

			assert reset_rs.success == True

	# asserts here will only work if this is the only way that nodes have been added to the tree
	def add_from_yaml(self, file_name, add_to="ROOT_0"):

		wd = pathlib.Path(__file__).parent.resolve()
		
		yaml_dict = None

		assert file_name.split('.')[-1] == 'yaml'

		with open(f'{wd}{file_name}', 'r') as file:
			yaml_dict = yaml.safe_load(file)

		assert yaml_dict != None

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD_FROM_FILE

		modify_rq.to_modify.append(add_to)
		modify_rq.to_add = f'{wd}{file_name}'

		modify_future = TestGoitrSimple.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

		fetch_rs = self.get_tree()

		node_names_orig = yaml_dict['NodeList']
		node_parents_orig = [ yaml_dict['Nodes'][i]['parent'] for i in node_names_orig ]

		node_names_from_server = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]
		parent_names_from_server = [ i.parent_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

		# verify that all nodes were added
		for index, val in enumerate(node_names_orig):
			assert val in node_names_from_server[index]

		# verify that each node has the correct parent
		for index, val in enumerate(node_parents_orig):
			assert val in parent_names_from_server[index] or (val == 'NONE' and parent_names_from_server[index] == add_to)

		goitr_names_from_server = [ i.goitr_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

		# verify goitrs were added correctly
		for index, val in enumerate(yaml_dict['Nodes']):
			try:
				goitr_type_from_file = i['goitr_type']

				assert goitr_type_from_file == goitr_names_from_server[index]
			except:
				continue

	def change_node_params(self, node_name, params):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.PARAM_UPDATE
		modify_rq.to_modify = [ node_name ]
		modify_rq.params = params

		modify_future = TestGoitrSimple.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def add_node(self, node, parent):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.to_modify = [ parent ]
		modify_rq.add_node = node

		modify_future = TestGoitrSimple.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def remove_node(self, node_name):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.to_modify = [ node_name ]

		modify_future = TestGoitrSimple.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	# node_name, params
	def change_node_params_from_goitr(self, node_name, params, goitr_client):
		
		goitr_rq = GoitrRequest.Request()

		goitr_rq.type = GoitrRequest.Request.CHANGE_PARAMS

		goitr_rq.params.append(node_name)

		for i in params:
			goitr_rq.params.append(i)

		goitr_future = goitr_client.call_async(goitr_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, goitr_future)

		goitr_rs = goitr_future.result()

		assert goitr_rs.success == True

	# parent_name, node_name, type, goitr_type, params
	def add_node_from_goitr(self, node, parent, goitr_client):
		
		goitr_rq = GoitrRequest.Request()

		goitr_rq.type = GoitrRequest.Request.ADD

		goitr_rq.params.append(parent)
		goitr_rq.params.append(node.node_name)
		goitr_rq.params.append(node.plugin_name)
		goitr_rq.params.append(node.goitr_name)

		for i in node.params:
			goitr_rq.params.append(i)

		goitr_future = goitr_client.call_async(goitr_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, goitr_future)

		goitr_rs = goitr_future.result()

		assert goitr_rs.success == True

	# node_name
	def remove_node_from_goitr(self, node_name, goitr_client):
		
		goitr_rq = GoitrRequest.Request()

		goitr_rq.type = GoitrRequest.Request.REMOVE

		goitr_rq.params.append(node_name)

		goitr_future = goitr_client.call_async(goitr_rq)
		rclpy.spin_until_future_complete(TestGoitrSimple.node, goitr_future)

		goitr_rs = goitr_future.result()

		assert goitr_rs.success == True

	def get_node_from_partial_name(self, node_name):
		tree = self.get_tree()

		for i in tree.found_subtrees[0].tree_nodes:
			if node_name in i.node_name:
				return i.node_name

	def wait_for_status_from(self, node_name):
		TestGoitrSimple.node.updated_states[node_name] = False

		while not TestGoitrSimple.node.updated_states[node_name]:
			pass

	def test_add_goitrs_from_yaml(self):

		with TestGoitrSimple.lock:
			self.initialize()
			self.reset_tree()

			self.add_from_yaml("/test_descriptions/test_simple_goitr.yaml")
