#!/usr/bin/python

import pytest
import rclpy
import rclpy.node
import pathlib
import yaml

from dhtt_msgs.srv import ModifyRequest, FetchRequest
from dhtt_msgs.msg import Subtree, Node

class ServerNode (rclpy.node.Node):

	def __init__(self):
		super().__init__('test_node')
		self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
		assert self.modifysrv.wait_for_service(timeout_sec=5.0)

		self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
		assert self.fetchsrv.wait_for_service(timeout_sec=5.0)

class TestServerAddRemove:

	first = True
	
	def initialize(self):
		if TestServerAddRemove.first:
			rclpy.init()

			TestServerAddRemove.first = False

		self.node = ServerNode()

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

			reset_rq = ModifyRequest.Request()
			reset_rq.type = ModifyRequest.Request.REMOVE

			for i in fetch_rs.found_subtrees[0].tree_nodes[0].children:
				reset_rq.to_modify.append(fetch_rs.found_subtrees[0].tree_nodes[i].node_name)

			reset_future = self.node.modifysrv.call_async(reset_rq)
			rclpy.spin_until_future_complete(self.node, reset_future)

			reset_rs = reset_future.result()

			assert reset_rs.success == True

			for i in reset_rs.removed_nodes:
				assert i in nodes_to_remove

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
		node_parents_orig = [yaml_dict['Nodes'][i]['parent'] for i in node_names_orig]

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

	def multiple_yaml(self, path):
		pass

	def get_modify_result(self, modify_rq):
		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		return modify_future.result()

	def test_remove_behavior(self):
		self.initialize()
		self.reset_tree()

		wd = pathlib.Path(__file__).parent.resolve()

		self.add_from_yaml('/test_descriptions/test_simple.yaml')

		fetch_rs = self.get_tree()

		to_remove = None

		for i in fetch_rs.found_subtrees[0].tree_nodes:
			if i.type == Node.BEHAVIOR:
				to_remove = i
				break

		modify_rq = ModifyRequest.Request()
		modify_rq.force = True;
		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.to_modify.append(to_remove.node_name)

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True
		assert to_remove.node_name in modify_rs.removed_nodes

		fetch_rs = self.get_tree()

		node_names_left = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes ]

		assert not to_remove.node_name in node_names_left  

	def test_remove_subtree(self):
		self.initialize()
		self.reset_tree()

		wd = pathlib.Path(__file__).parent.resolve()

		self.add_from_yaml('/test_descriptions/test_simple.yaml')

		fetch_rs = self.get_tree()

		to_remove = None

		for i in fetch_rs.found_subtrees[0].tree_nodes:
			if i.type == Node.AND:
				to_remove = i
				break

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.force = True;
		modify_rq.to_modify.append(to_remove.node_name)

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == True
		assert to_remove.node_name in modify_rs.removed_nodes

		fetch_rs = self.get_tree()

		node_names_left = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes ]
		parent_names_left = [ i.parent_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

		assert not to_remove.node_name in node_names_left 
		assert not to_remove.node_name in parent_names_left

	def test_remove_root(self):
		self.initialize()
		self.reset_tree()

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.force = True;
		modify_rq.to_modify.append('ROOT_0')

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == False
		assert not 'ROOT_0' in modify_rs.removed_nodes
		assert modify_rs.error_msg != ""

	def test_remove_bad_params(self):
		self.initialize()
		self.reset_tree()

		# first check that nodes which don't exist can't be removed
		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.force = True;
		modify_rq.to_modify.append('ROOT_23')

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == False
		assert not 'ROOT_23' in modify_rs.removed_nodes
		assert modify_rs.error_msg != ""

		# next check that the right message type has to be sent through
		modify_rq.to_modify = []
		modify_rq.type = 23
		modify_rq.to_modify.append('ROOT_0')

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == False
		assert not 'ROOT_0' in modify_rs.removed_nodes
		assert modify_rs.error_msg != ""

		# now check that nodes can't be double removed
		modify_rq.type = ModifyRequest.Request.REMOVE

		wd = pathlib.Path(__file__).parent.resolve()

		self.add_from_yaml('/test_descriptions/test_simple.yaml')

		fetch_rs = self.get_tree()

		to_remove = None

		for i in fetch_rs.found_subtrees[0].tree_nodes:
			if i.type == Node.AND:
				to_remove = i
				break

		modify_rq.to_modify = []
		modify_rq.to_modify.append(to_remove.node_name)
		modify_rq.to_modify.append(to_remove.node_name)

		modify_future = self.node.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self.node, modify_future)

		modify_rs = modify_future.result()

		assert modify_rs.success == False
		assert to_remove.node_name in modify_rs.removed_nodes
		assert modify_rs.error_msg != ""

	def test_add_manually(self):
		self.initialize()
		self.reset_tree()

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.force = True;

		modify_rq.to_modify.append('ROOT_0')

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.AND
		modify_rq.add_node.node_name = 'ParentAnd'
		modify_rq.add_node.plugin_name = 'dhtt_plugins::AndBehavior' # just for now

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == True
		assert 'ParentAnd' in modify_rs.added_nodes[0]
		assert modify_rs.error_msg == ''

	def test_add_parent_dne(self):
		self.initialize()
		self.reset_tree()

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.force = True;

		modify_rq.to_modify.append('ROOT_23')

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.AND
		modify_rq.add_node.node_name = 'ParentAnd'
		modify_rq.add_node.plugin_name = 'dhtt_plugins::TestBehavior'

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == False
		assert len(modify_rs.added_nodes) == 0
		assert modify_rs.error_msg != ''

	def test_add_bad_params(self):
		self.initialize()
		self.reset_tree()

		# will fill this in when I get to the class structure and plugins

	def test_add_malformed_node(self):
		self.initialize()
		self.reset_tree()

		# check when the node type is not possible
		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.force = True;

		modify_rq.to_modify.append('ROOT_23')

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.BEHAVIOR + 1
		modify_rq.add_node.node_name = 'ParentFakeBehavior'
		modify_rq.add_node.plugin_name = 'dhtt::WhoCouldItbe'

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == False
		assert len(modify_rs.added_nodes) == 0
		assert modify_rs.error_msg != ''

		# check when the plugin is nonexistant (easier once I make the class structure)

	def test_add_node_mismatch_typeplugin(self):
		self.initialize()
		self.reset_tree()

		root_name = self.get_tree().found_subtrees[0].tree_nodes[0].node_name

		# check when the node type is not possible
		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD

		modify_rq.to_modify.append(root_name)

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.AND
		modify_rq.add_node.node_name = 'MismatchedTypeAndPlugin'
		modify_rq.add_node.plugin_name = 'dhtt_plugins::OrBehavior'

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == False
		assert len(modify_rs.added_nodes) == 0
		assert modify_rs.error_msg != ''

	def test_add_to_behavior(self):
		self.initialize()
		self.reset_tree()

		wd = pathlib.Path(__file__).parent.resolve()

		self.add_from_yaml('/test_descriptions/test_simple.yaml')

		fetch_rs = self.get_tree()

		behavior_add_to = None

		for i in fetch_rs.found_subtrees[0].tree_nodes:
			if i.type == Node.BEHAVIOR:
				behavior_add_to = i
				break

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.force = True;

		modify_rq.to_modify.append(behavior_add_to.node_name)

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.BEHAVIOR
		modify_rq.add_node.node_name = 'BadChildBehavior'
		modify_rq.add_node.plugin_name = 'dhtt_plugins::TestBehavior'

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == False
		assert len(modify_rs.added_nodes) == 0
		assert modify_rs.error_msg != ''

	def test_huge_add(self):
		self.initialize()
		self.reset_tree()

		# gonna leave blank and passing for now

	def test_add_new_root(self):
		self.initialize()
		self.reset_tree()

		modify_rq = ModifyRequest.Request()
		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.force = True;

		modify_rq.to_modify.append('ROOT_0')

		modify_rq.add_node = Node()
		modify_rq.add_node.type = Node.ROOT
		modify_rq.add_node.node_name = 'BadRootBehavior'
		modify_rq.add_node.plugin_name = 'dhtt_plugins::TestBehavior'

		modify_rs = self.get_modify_result(modify_rq)

		assert modify_rs.success == False
		assert len(modify_rs.added_nodes) == 0
		assert modify_rs.error_msg != ''

	def test_add_from_file(self):
		self.initialize()
		self.reset_tree()

		wd = pathlib.Path(__file__).parent.resolve()

		self.add_from_yaml('/test_descriptions/test_simple.yaml')
		
		self.reset_tree()

		self.multiple_yaml(wd)

	# def test_add_with_file_args(self):
	# 	self.initialize()
	# 	self.reset_tree()

	# 	modify_rq = ModifyRequest.Request()
	# 	modify_rq.type = ModifyRequest.Request.ADD
	# 	modify_rq.force = True;

	# 	modify_rq.to_modify.append('ROOT_0')

	# 	modify_rq.add_node = Node()
	# 	modify_rq.add_node.type = Node.THEN
	# 	modify_rq.add_node.node_name = 'ParentThen'
	# 	modify_rq.add_node.plugin_name = 'dhtt_plugins::ThenBehavior' # just for now

	# 	modify_rs = self.get_modify_result(modify_rq)

	# 	assert modify_rs.success == True
	# 	assert 'ParentThen' in modify_rs.added_nodes[0]
	# 	assert modify_rs.error_msg == ''

	# 	wd = pathlib.Path(__file__).parent.parent.resolve()

	# 	self.add_from_yaml('/sample_tasks/pick_place.yaml', add_to=modify_rs.added_nodes[0], file_args=[ 'target: object1', 'pick_spot: loc1', 'place_spot: loc2' ], start_index=2)