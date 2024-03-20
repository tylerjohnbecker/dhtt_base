#!/usr/bin/python3

import rclpy
import rclpy.node
import pathlib
import yaml

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

	def get_tree(self):
		fetch_rq = FetchRequest.Request()
		fetch_rq.return_full_subtree = True

		fetch_future = self.fetchsrv.call_async(fetch_rq)
		rclpy.spin_until_future_complete(self, fetch_future)

		fetch_rs = fetch_future.result()

		return fetch_rs

	def reset_tree(self):

		fetch_rs = self.get_tree()

		if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:

			nodes_to_remove = [ i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:] ]

			reset_rq = ControlRequest.Request()
			reset_rq.type = ControlRequest.Request.RESET

			reset_future = self.controlsrv.call_async(reset_rq)
			rclpy.spin_until_future_complete(self, reset_future)

			reset_rs = reset_future.result()

	def start_tree(self):
		control_rq = ControlRequest.Request()
		control_rq.type = ControlRequest.Request.START

		control_future = self.controlsrv.call_async(control_rq)
		rclpy.spin_until_future_complete(self, control_future)

		control_rs = control_future.result()

		return control_rs

	def wait_for_node_in_state(self, node_name, state):
		while self.node_states[node_name].node_status.state != state and rclpy.ok():
			rclpy.spin_once(self)

	def mutate_node_to_type(self, node_name, mutate_type):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.MUTATE
		modify_rq.to_modify = [ node_name ]
		modify_rq.mutate_type = mutate_type

		modify_future = self.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self, modify_future)

		modify_rs = modify_future.result()

	def add_node(self, node, parent):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.ADD
		modify_rq.to_modify = [ parent ]
		modify_rq.add_node = node

		modify_future = self.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self, modify_future)

		modify_rs = modify_future.result()

	def remove_node(self, node_name):

		modify_rq = ModifyRequest.Request()

		modify_rq.type = ModifyRequest.Request.REMOVE
		modify_rq.to_modify = [ node_name ]

		modify_future = self.modifysrv.call_async(modify_rq)
		rclpy.spin_until_future_complete(self, modify_future)

		modify_rs = modify_future.result()

	def get_node_from_partial_name(self, node_name):
		tree = self.get_tree()

		for i in tree.found_subtrees[0].tree_nodes:
			if node_name in i.node_name:
				return i.node_name

	def add_pick_place_to(self, node_name, pickup_point, place_point, activation_potential, parent_node):
		# create node messages
		then_node = Node()

		then_node.node_name = node_name
		then_node.plugin_name = "dhtt_plugins::ThenBehavior"
		then_node.params = []
		then_node.type = Node.THEN

		move_node_1 = Node()

		move_node_1.node_name = f'moveToPickup{node_name[-1:]}'
		move_node_1.plugin_name = "dhtt_plugins::MoveBehavior"
		move_node_1.params = [f'activation_potential: {activation_potential}', f'dest: {pickup_point}']
		move_node_1.type = Node.BEHAVIOR

		pick_node = Node()

		pick_node.node_name = f'pickupObject{node_name[-1:]}'
		pick_node.plugin_name = "dhtt_plugins::PickBehavior"
		pick_node.params = [f'activation_potential: {activation_potential}', 'object: any']
		pick_node.type = Node.BEHAVIOR

		move_node_2 = Node()

		move_node_2.node_name =f'moveToDestination{node_name[-1:]}'
		move_node_2.plugin_name = "dhtt_plugins::MoveBehavior"
		move_node_2.params = [f'activation_potential: {activation_potential}', f'dest: {place_point}']
		move_node_2.type = Node.BEHAVIOR

		place_node = Node()

		place_node.node_name = f'placeObject{node_name[-1:]}'
		place_node.plugin_name = "dhtt_plugins::PlaceBehavior"
		place_node.params = [f'activation_potential: {activation_potential}']
		place_node.type = Node.BEHAVIOR

		# add parent then to the tree
		self.add_node(then_node, parent_node)

		# get assigned name
		assigned_name = self.get_node_from_partial_name(node_name)

		self.get_logger().info(f'adding children to node {assigned_name}')

		# add behaviors to assigned name in order
		self.add_node(move_node_1, assigned_name)
		self.add_node(pick_node, assigned_name)
		self.add_node(move_node_2, assigned_name)
		self.add_node(place_node, assigned_name)

	def run_experiment(self):

		self.reset_tree()

		topmost_then = Node()

		topmost_then.node_name = "Topmost_Then"
		topmost_then.plugin_name = "dhtt_plugins::ThenBehavior"
		topmost_then.type = Node.THEN

		self.add_node(topmost_then, 'ROOT_0')

		assigned_name = self.get_node_from_partial_name('Topmost_Then')

		self.add_pick_place_to('PickPlace1', 'home', 'depot_1', 10, assigned_name)
		self.add_pick_place_to('PickPlace2', 'home', 'depot_3', 5, assigned_name)
		self.add_pick_place_to('PickPlace3', 'home', 'depot_5', 15, assigned_name)

		self.start_tree()

		self.wait_for_node_in_state('placeObject1', NodeStatus.WORKING)

		self.mutate_node_to_type(assigned_name, "dhtt_plugins::AndBehavior")

		self.add_pick_place_to('PickPlace4', 'home', 'depot_2', 7, assigned_name)
		self.add_pick_place_to('PickPlace5', 'home', 'depot_4', 5, assigned_name)

		self.wait_for_node_in_state('PickPlace3', NodeStatus.WORKING)

		assigned_name = self.get_node_from_partial_name('PickPlace2')

		self.remove_node(assigned_name)

		tree_list = [ i.node_name for i in self.get_tree().found_subtrees[0].tree_nodes ]

		self.get_logger().info(f'{tree_list}')

def main():
	rclpy.init()

	srv = ServerNode()

	srv.get_logger().info("HELLLLOOOO")

	srv.run_experiment()

if __name__ == '__main__':
	main()