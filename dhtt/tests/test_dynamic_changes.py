#!/usr/bin/python

import pytest
import rclpy
import rclpy.node
import pathlib
import yaml
import copy

from threading import Lock

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
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

	def root_status_listener(self, data):
		self.root_state = data.state

	def status_listener(self, data):
		node_name = data.node_name.split('_')[0]

		self.node_states[node_name] = data

class TestDynamicChanges:

	first = True
	node = None
	lock = Lock()
	
	def initialize(self):

		if TestDynamicChanges.first:
			rclpy.init()

			TestDynamicChanges.first = False

			TestDynamicChanges.node = ServerNode()

		self.ok = False

	def get_tree(self):
		fetch_rq = FetchRequest.Request()
		fetch_rq.return_full_subtree = True

		fetch_future = TestDynamicChanges.node.fetchsrv.call_async(fetch_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, fetch_future)

		fetch_rs = fetch_future.result()

		assert fetch_rs.success == True

		return fetch_rs

	def reset_tree(self):

		TestDynamicChanges.node.node_states = {}

		fetch_rs = self.get_tree()

		if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:

			nodes_to_remove = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

			reset_rq = ControlRequest.Request()
			reset_rq.type = ControlRequest.Request.RESET

			reset_future = TestDynamicChanges.node.controlsrv.call_async(reset_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, reset_future)

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

		modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

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

		control_future = TestDynamicChanges.node.controlsrv.call_async(control_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, control_future)

		control_rs = control_future.result()

		assert control_rs.success == True

		return control_rs

	def interrupt_tree(self):
		control_rq = ControlRequest.Request()
		control_rq.type = ControlRequest.Request.STOP

		control_future = TestDynamicChanges.node.controlsrv.call_async(control_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, control_future)

		control_rs = control_future.result()

		assert control_rs.success == True

	def wait_for_node_in_state(self, node_name, state):
		while TestDynamicChanges.node.node_states[node_name].node_status.state != state and rclpy.ok():
			rclpy.spin_once(TestDynamicChanges.node)
			
		assert rclpy.ok()

	def wait_for_waiting(self):
		while TestDynamicChanges.node.root_state != NodeStatus.WAITING and rclpy.ok():
			rclpy.spin_once(TestDynamicChanges.node)
			
		assert rclpy.ok()

	def wait_for_finished_execution(self):

		while TestDynamicChanges.node.root_state != NodeStatus.DONE and rclpy.ok():
			rclpy.spin_once(TestDynamicChanges.node)

		assert rclpy.ok()


	def get_history(self):
		history_rq = HistoryRequest.Request()

		history_future = TestDynamicChanges.node.historysrv.call_async(history_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, history_future)

		history_rs = history_future.result()

		return history_rs.node_history

	def compare_history_to_expected(self, history, expected):
		assert len(history) == len(expected)

		for index, val in enumerate(expected):
			assert val == history[index].split('_')[0]

	def mutate_node_to_type(self, node_name, mutate_type):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.MUTATE
		modify_rq.to_modify = [ node_name ]
		modify_rq.mutate_type = mutate_type

		modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def change_node_params(self, node_name, params):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.PARAM_UPDATE
		modify_rq.to_modify = [ node_name ]
		modify_rq.params = params

		modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def add_node(self, node, parent):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.to_modify = [ parent ]
		modify_rq.add_node = node

		modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def remove_node(self, node_name):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.to_modify = [ node_name ]

		modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def get_node_from_partial_name(self, node_name):
		tree = self.get_tree()

		for i in tree.found_subtrees[0].tree_nodes:
			if node_name in i.node_name:
				return i.node_name

	def wait_for_type_change(self, node_name, original_type):
		while TestDynamicChanges.node.node_states[node_name].plugin_name == original_type:
			rclpy.spin_once(TestDynamicChanges.node)
			assert rclpy.ok()

	def test_interrupt(self):

		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()

			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# start tree
			self.start_tree()

			# interrupt at some specified state
			self.wait_for_node_in_state("ParentAnd", NodeStatus.WORKING)

			self.interrupt_tree()

			# verify that it stopped in the correct spot
			self.wait_for_waiting()

			assert TestDynamicChanges.node.root_state == NodeStatus.WAITING

			# restart
			self.start_tree()

			self.wait_for_finished_execution()

			# check history
			history = self.get_history()
			expected = self.load_expected_from_yaml("/test_descriptions/simple_and.yaml")

			self.compare_history_to_expected(history, expected)

	def test_static_mutate(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentAnd")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentAnd"].plugin_name)

			self.mutate_node_to_type(to_mutate, "dhtt_plugins::ThenBehavior")

			assert original_type == "dhtt_plugins::AndBehavior"

			self.wait_for_type_change("ParentAnd", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentAnd"].plugin_name == "dhtt_plugins::ThenBehavior"

			# start tree
			self.start_tree()

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask", "ThirdTask", "SecondTask"]

			self.compare_history_to_expected(history, expected)

	def test_static_param_change(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# change params ( activation potential )
			to_change = self.get_node_from_partial_name("FirstTask")
			self.change_node_params(to_change, [ "activation_potential: 2.5" ])

			assert TestDynamicChanges.node.node_states["FirstTask"].params[0] == "activation_potential: 2.5"

			# run tree
			self.start_tree()

			self.wait_for_finished_execution()

			# verify updated history
			history = self.get_history()
			expected = ["SecondTask", "FirstTask", "ThirdTask"]

			self.compare_history_to_expected(history, expected)

	def test_dynamic_add(self):

		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# create node to add
			to_add = Node()
			to_add.node_name = "NewNode"
			to_add.params = ["activation_potential: 7.5"]

			to_add.type = Node.BEHAVIOR
			to_add.plugin_name = "dhtt_plugins::TestBehavior"

			parent = self.get_node_from_partial_name("ParentAnd")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentAnd", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_waiting()

			# dynamically add node
			self.add_node(to_add, parent)

			self.start_tree()

			# wait until execution finished
			self.wait_for_finished_execution()

			history = self.get_history()
			expected = ["FirstTask", "NewNode", "SecondTask", "ThirdTask"]

			self.compare_history_to_expected(history, expected)

	def test_dynamic_remove(self):
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# create node to add
			to_remove = self.get_node_from_partial_name("SecondTask")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentAnd", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_waiting()

			# dynamically add node
			self.remove_node(to_remove)
			
			self.start_tree()

			# wait until execution finished
			self.wait_for_finished_execution()

			history = self.get_history()
			expected = ["FirstTask", "ThirdTask"]

			self.compare_history_to_expected(history, expected)

	def test_dynamic_and_to_or(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_and.yaml")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentAnd", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_waiting()

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentAnd")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentAnd"].plugin_name)

			assert original_type == "dhtt_plugins::AndBehavior"

			self.mutate_node_to_type(to_mutate, "dhtt_plugins::OrBehavior")

			self.wait_for_type_change("ParentAnd", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentAnd"].plugin_name == "dhtt_plugins::OrBehavior"

			# start tree
			self.start_tree()

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask"]

			self.compare_history_to_expected(history, expected)

		pass

	def test_dynamic_then_to_and(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_then.yaml")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentThen", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_waiting()

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentThen")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentThen"].plugin_name)

			assert original_type == "dhtt_plugins::ThenBehavior"

			self.mutate_node_to_type(to_mutate, "dhtt_plugins::AndBehavior")

			self.wait_for_type_change("ParentThen", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentThen"].plugin_name == "dhtt_plugins::AndBehavior"

			# start tree
			self.start_tree()

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask", "ThirdTask", "SecondTask"]

			self.compare_history_to_expected(history, expected)

		pass

	def test_dynamic_then_to_or(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_then.yaml")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentThen", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_waiting()

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentThen")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentThen"].plugin_name)

			assert original_type == "dhtt_plugins::ThenBehavior"
			
			self.mutate_node_to_type(to_mutate, "dhtt_plugins::OrBehavior")

			self.wait_for_type_change("ParentThen", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentThen"].plugin_name == "dhtt_plugins::OrBehavior"

			# start tree
			self.start_tree()

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask"]

			self.compare_history_to_expected(history, expected)

		pass

	def test_dynamic_or_to_and(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_or.yaml")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentOr", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_finished_execution()

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentOr")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentOr"].plugin_name)

			assert original_type == "dhtt_plugins::OrBehavior"
			
			self.mutate_node_to_type(to_mutate, "dhtt_plugins::AndBehavior")

			self.wait_for_type_change("ParentOr", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentOr"].plugin_name == "dhtt_plugins::AndBehavior"

			# start tree
			self.start_tree()

			TestDynamicChanges.node.root_state = 0

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask", "SecondTask", "ThirdTask"]

			self.compare_history_to_expected(history, expected)

		pass

	def test_dynamic_or_to_then(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
		
			# add yaml to tree
			self.add_from_yaml("/test_descriptions/simple_or.yaml")

			# start tree
			self.start_tree()

			self.wait_for_node_in_state("ParentOr", NodeStatus.WORKING)

			self.interrupt_tree()

			self.wait_for_finished_execution()

			# mutate task node
			to_mutate = self.get_node_from_partial_name("ParentOr")
			original_type = copy.copy(TestDynamicChanges.node.node_states["ParentOr"].plugin_name)

			assert original_type == "dhtt_plugins::OrBehavior"
			
			self.mutate_node_to_type(to_mutate, "dhtt_plugins::ThenBehavior")

			self.wait_for_type_change("ParentOr", original_type)

			# verify that it worked
			assert TestDynamicChanges.node.node_states["ParentOr"].plugin_name == "dhtt_plugins::ThenBehavior"

			# start tree
			self.start_tree()

			TestDynamicChanges.node.root_state = 0

			self.wait_for_finished_execution()

			# verify that the order is as expected
			history = self.get_history()
			expected = ["FirstTask", "ThirdTask", "SecondTask"]

			self.compare_history_to_expected(history, expected)

		pass

	def test_complex_dynamic_tree(self):
		
		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()

			# run experiment 3 and verify that it works

		pass

	def test_reparent(self):

		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
			self.add_from_yaml("/test_descriptions/then_parent_and.yaml")
			# self.start_tree()
			# self.wait_for_node_in_state("ParentThen", NodeStatus.WORKING)
			# self.interrupt_tree()
			# self.wait_for_waiting()

			# reparent
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["SecondTask_4"]  # was under MidParentAnd_3
			modify_rq.new_parent = "ParentThen_1"

			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)

			modify_rs = modify_future.result()
			assert modify_rs.success

			assert [(x.node_name, x.parent_name) for x in self.get_tree().found_subtrees[0].tree_nodes] == [('ROOT_0', 'NONE'), ('ParentThen_1', 'ROOT_0'),
                                                                                                   ('FirstTask_2', 'ParentThen_1'), ('MidParentAnd_3', 'ParentThen_1'), ('SecondTask_4', 'ParentThen_1'), ('ThirdTask_5', 'MidParentAnd_3')]

			self.start_tree()

			# wait until execution finished
			self.wait_for_finished_execution()

			history = self.get_history()
			expected = ["FirstTask", "NewNode", "SecondTask", "ThirdTask"]
			# self.compare_history_to_expected(history, expected)

			self.reset_tree()

	def test_reparent_negatives(self):

		with TestDynamicChanges.lock:
			self.initialize()
			self.reset_tree()
			self.add_from_yaml("/test_descriptions/then_parent_and.yaml")
			self.start_tree()
			self.wait_for_node_in_state("ParentThen", NodeStatus.WORKING)
			self.interrupt_tree()
			self.wait_for_waiting()

			# reparent root
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["ROOT_0"]
			modify_rq.new_parent = "ParentThen_1"
			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)
			assert modify_future.result().success == False

			# behavior node parent
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["SecondTask_4"]
			modify_rq.new_parent = "ThirdTask_5"
			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)
			assert modify_future.result().success == False

			# nonexistant to_modify
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["foobar"]
			modify_rq.new_parent = "ParentThen_1"
			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)
			assert modify_future.result().success == False

			# nonexistant new_parent
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["SecondTask_4"]
			modify_rq.new_parent = "foobar"
			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)
			assert modify_future.result().success == False

			# reparent to self
			modify_rq = ModifyRequest.Request()
			modify_rq.type = ModifyRequest.Request.REPARENT
			modify_rq.to_modify = ["SecondTask_4"]
			modify_rq.new_parent = "SecondTask_4"
			modify_future = TestDynamicChanges.node.modifysrv.call_async(modify_rq)
			rclpy.spin_until_future_complete(TestDynamicChanges.node, modify_future)
			assert modify_future.result().success == False

			self.reset_tree()
