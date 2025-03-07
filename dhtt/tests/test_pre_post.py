#!/usr/bin/python3

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
    with lock.acquire(poll_intervall=0.1):
        yield

# add system for compairing preconditions and postconditions
class PredicateConjunction:

	def __init__ (self, str=""):
		self.predicates = []
		self.conjunctions = []
		self.operator = '^'

		self.parse_from_string(str)

	# copy of convert_to_struct in utils.cc
	def parse_from_string(self, input_str):

		# hide recursive function
		def parse_helper(to_build, current_str):
			
			looking_for = '-'
			scope_start = -1
			op = ''

			for index, val in enumerate(list(current_str)):

				if looking_for == '-' and op == '' and val == '^':
					op = '^'

				elif looking_for == '-' and op == '' and val == 'v':
					op = 'v'

				elif val == '(' and looking_for == '-':
					scope_start = index + 1
					looking_for = ')'

				elif val == ')' and looking_for == ')':
					to_build.predicates.append(current_str[scope_start: index])
					looking_for = '-'

				elif val == '[' and looking_for == '-':
					scope_start = index + 1
					looking_for = ']'

				elif val == ']' and looking_for == ']':
					n_conjunction = PredicateConjunction()

					parse_helper(n_conjunction, current_str[scope_start: index])

					to_build.conjunctions.append(n_conjunction)
					looking_for = '-'

			to_build.operator = op

			return

		# delete old data
		self.predicates = []
		self.conjunctions = []

		parse_helper(self, input_str)

	def sort_alphabetically(self):
		self.predicates.sort() 

		for i in self.conjunctions:
			i.sort_alphabetically()

	def convert_to_string(self):
		
		def to_string_rec(cur_conj):

			to_ret = ""

			for i in cur_conj.predicates[:-1]: 
				to_ret += "(" + i + ")" + cur_conj.operator

			if len(cur_conj.predicates) > 0:
				to_ret += "(" + cur_conj.predicates[-1] + ")"

			for i in cur_conj.conjunctions:
				to_ret += cur_conj.operator + "[" + to_string_rec(i) + "]"

			return to_ret

		# recursively sort so that the string is unique
		self.sort_alphabetically()

		return to_string_rec(self)

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

		self.debug_pub = self.create_publisher(String, "/debug_out", 10)

		self.waiting = True
		self.success = False

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

	def simple_wait(self, data):
		out = 'DING'

		out_msg = String()
		out_msg.data = out

		self.debug_pub.publish( out_msg )

		self.waiting = False
		self.success = data.success

class TestGoitrSimple:
	
	rclpy.init()
	
	def initialize(self):
		self.node = ServerNode()

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

		self.node.node_states = {}

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

	def get_expected_conditions(self, file_name):

		wd = pathlib.Path(__file__).parent.resolve()

		yaml_dict = None

		assert file_name.split('.')[-1] == 'yaml'

		with open(f'{wd}{file_name}', 'r') as file:
			yaml_dict = yaml.safe_load(file)

		assert yaml_dict != None

		assert 'expected_preconditions' in yaml_dict.keys() and 'expected_postconditions' in yaml_dict.keys()

		return [ yaml_dict['expected_preconditions'], yaml_dict['expected_postconditions'] ]

	def change_node_params(self, node_name, params):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.PARAM_UPDATE
		modify_rq.to_modify = [ node_name ]
		modify_rq.params = params

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def add_node(self, node, parent, index=-1, pass_check=True):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.to_modify = [ parent ]
		modify_rq.add_node = node
		modify_rq.force = False
		modify_rq.index = index

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == pass_check

	def remove_node(self, node_name):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.to_modify = [ node_name ]

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True

	def start_tree(self):
		control_rq = ControlRequest.Request()
		control_rq.type = ControlRequest.Request.START

		control_future = self.node.controlsrv.call_async(control_rq)
		rclpy.spin_until_future_complete(self.node, control_future)

		control_rs = control_future.result()

		assert control_rs.success == True

		return control_rs

	def get_node_from_partial_name(self, node_name):
		tree = self.get_tree()

		for i in tree.found_subtrees[0].tree_nodes:
			if node_name in i.node_name:
				return i.node_name

	def wait_for_status_from(self, node_name):
		self.node.updated_states[node_name] = False

		while not self.node.updated_states[node_name]:
			pass

	def test_simple_and_conditions_unique(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_and_unique_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_and_unique_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_simple_and_conditions_duplicated(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_and_duplicate_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_and_duplicate_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_simple_or_conditions_unique(self, serial):

		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_or_unique_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_or_unique_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_simple_or_conditions_duplicated(self, serial):

		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_or_duplicate_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_or_duplicate_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_simple_then_conditions_unique(self, serial):

		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_then_unique_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_then_unique_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_simple_then_conditions_leading(self, serial):

		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_then_leading_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_then_leading_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_and_parent_or_conditions_unique(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/and_parent_or_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/and_parent_or_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_and_parent_then_conditions_unique(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/and_parent_then_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/and_parent_then_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_or_parent_and_conditions_unique(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/or_parent_and_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/or_parent_and_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_or_parent_then_unique(self, serial):	
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/or_parent_then_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/or_parent_then_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()

	def test_then_parent_and_unique(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/then_parent_and_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/then_parent_and_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()	

	def test_then_parent_or_unique(self, serial):

		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/then_parent_or_conditions.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/then_parent_or_conditions.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()	

	def test_then_parent_and_leading(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/then_parent_and_conditions_leading.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/then_parent_and_conditions_leading.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()	

	def test_then_parent_or_leading(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/then_parent_or_conditions_leading.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/then_parent_or_conditions_leading.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()	

	def test_complex_tree_conditions(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/complex_tree.yaml", force=True)

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/complex_tree.yaml")

		response = self.get_tree()

		expected_conditions = [ PredicateConjunction(exp_pre), PredicateConjunction(exp_post) ]
		obs_conditions = [ PredicateConjunction(response.found_subtrees[0].tree_nodes[1].preconditions) , PredicateConjunction(response.found_subtrees[0].tree_nodes[1].postconditions) ]

		assert expected_conditions[0].convert_to_string() == obs_conditions[0].convert_to_string()
		assert expected_conditions[1].convert_to_string() == obs_conditions[1].convert_to_string()	

	def test_can_add_single_level(self, serial):
		
		self.initialize()
		self.reset_tree()

		added_nodes = self.add_from_yaml("/test_descriptions/pre_post/simple_then_leading_conditions.yaml", force=True)

		assert len(added_nodes) > 0

		exp_pre, exp_post = self.get_expected_conditions("/test_descriptions/pre_post/simple_then_leading_conditions.yaml")

		# the tree shouldn't add a node which would violate the preconditions of the last behavior
		wrong_node = Node()
		wrong_node.node_name = "no_add"
		wrong_node.parent_name = 'NONE'
		wrong_node.params = ['activation_potential: .4', '!a: b'] 
		wrong_node.plugin_name = 'dhtt_plugins::TestBehavior'
		wrong_node.type = Node.BEHAVIOR

		self.add_node(wrong_node, added_nodes[0], 1, False)

		wrong_node2 = Node()
		wrong_node2.node_name = "yes_add"
		wrong_node2.parent_name = 'NONE'
		wrong_node2.params = ['activation_potential: .4', 'a: b'] 
		wrong_node2.plugin_name = 'dhtt_plugins::TestBehavior'
		wrong_node2.type = Node.BEHAVIOR

		self.add_node(wrong_node2, added_nodes[0], 1, False)

		right_node = Node()
		right_node.node_name = "yes_add"
		right_node.parent_name = 'NONE'
		right_node.params = ['activation_potential: .4', 'c: e'] 
		right_node.plugin_name = 'dhtt_plugins::TestBehavior'
		right_node.type = Node.BEHAVIOR

		self.add_node(right_node, added_nodes[0], 1, True)


	def test_can_add_multi_level(self, serial):
		pass

	def test_can_remove(self, serial):
		pass

	def test_can_mutate(self, serial):
		pass

	def test_maintenance_time_to_compute(self, serial):
		pass