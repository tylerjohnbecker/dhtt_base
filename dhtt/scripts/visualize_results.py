#!/usr/bin/python3

import rclpy
import rclpy.node
import pathlib
import yaml
import sys
import matplotlib.pyplot as plt

from threading import Lock

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
from dhtt_msgs.msg import Subtree, Node, NodeStatus 

class ServerNode (rclpy.node.Node):

	def __init__(self):
		super().__init__('recording_node')

		self.root_state = 0

		# holds a key val pair for each node's state
		# node_name: [ start_time, end_time ]
		self.node_states = {}
		self.running = False

		self.root_status_sub = self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)
		self.status_sub = self.create_subscription(Node, "/status", self.status_listener, 10)

		self.max_time = -1


	def root_status_listener(self, data):
		self.root_state = data.state

	def status_listener(self, data):
		# just record the initial start time and the final end time
		if not data.node_name in self.node_states.keys() and data.node_status.state == NodeStatus.WORKING:
			self.node_states[data.node_name] = [ data ] 
		elif data.node_status.state == NodeStatus.DONE:
			self.node_states[data.node_name].append(data)

		# if data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec > self.max_time:
		# 	self.max_time = data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec

		# if not self.running:
		# 	return

		# # don't record the root node
		# if node_name == 'ROOT':
		# 	return

		# # if this is the first time we've heard from this node initialize it in the data structure
		# if not node_name in self.node_states.keys():
		# 	self.node_states[node_name] = [ [], [], [] ]
		# 	self.node_states[node_name][0].append(data.node_status.state)
		# 	self.node_states[node_name][1].append(data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec)
		# 	self.node_states[node_name][2].append(0)

		# 	return

		# # otherwise only add another value if the previous and current state are different
		# if self.node_states[node_name][0][-1] != data.node_status.state:
		# 	# width is final time minus initial time
		# 	self.node_states[node_name][2][-1] = data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec - self.node_states[node_name][1][-1]

		# 	self.node_states[node_name][0].append(data.node_status.state)
		# 	self.node_states[node_name][1].append(data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec)
		# 	self.node_states[node_name][2].append(0)

	def get_time_float(self, data):
		return data.head.stamp.sec + 10e-9 * data.head.stamp.nanosec

	def insert_sorted_by_val(self, sorted_list, val_to_insert, original_index):

		for index, val in enumerate(sorted_list):
			# this is when we insert
			if val[0] < val_to_insert:
				sorted_list.insert(index, [val_to_insert, original_index])

		sorted_list.append([val_to_insert, original_index])

	# def get_nodes_of_depth(self, depth_range):
	# 	pass

	def sort_by_given_indices(self, to_sort, given_indices):

		to_ret = []

		for i in given_indices:
			to_ret.append(to_sort[i])

		to_sort = to_ret


	# the data will become: [ data_start, data_end, [ start_time, end_time, width, row ] ]
	def get_statistics(self, nodes):

		# assume we passed in at least one element
		min_time = self.get_float_time(nodes[0][0])
		max_time = self.get_float_time(nodes[0][1])

		# first generate width, start_time, min_time, and max_time
		final_order = []
		for i, val in nodes[1:]:
			start_time = self.get_float_time(val[0])
			end_time = self.get_float_time(val[1])

			width = end_time - start_time

			val.append([start_time, end_time, width])

			# sort them by start time
			self.insert_sorted_by_val(final_order, start_time, i)

			if min_time > start_time:
				min_time = start_time

			if max_time < end_time:
				max_time = end_time

		# now sort them by their start time
		self.sort_by_given_indices(nodes, final_order[:][1])

		# assign each a row greedily
		# if there is no overlap then delete a row and move it up
		final_rows_list = [ [] for i in range(len(nodes)) ]
		for index, val in enumerate( nodes ):
			for i in range(len(final_rows_list)):
				if len(final_rows_list[i]) == 0: # if we hit an empty row just append it here ( we always do because there are n rows and n items)
					final_rows_list[i].append(val)
				elif final_rows_list[i][-1][2][1] < val[2][0]: # compare end time of the end of the row to start time of the new member
					final_rows_list[i].append(val)

		# finally get rid of any empty rows
		to_delete = -1
		for i in range(len(final_rows_list)):
			if len(final_rows_list[i]) == 0:
				to_delete = i

				break

		return final_rows_list[:i]


	def record_results(self):
		while self.root_state != NodeStatus.ACTIVE and rclpy.ok():
			rclpy.spin_once(self)

		if not rclpy.ok():
			sys.exit(0)

		self.running = True

		while self.root_state != NodeStatus.DONE and rclpy.ok():
			rclpy.spin_once(self)

	def graph_results(self):
		
		fig = plt.figure()

		ax = fig.add_subplot(111)

		height = .05

		height_iter = .1

		color_dict = {NodeStatus.WAITING: '#6485a1', NodeStatus.ACTIVE: '#cc0063', NodeStatus.WORKING: '#2c1fbf', NodeStatus.MUTATING: '#06cf17', NodeStatus.DONE: '#76138a'}

		for key in self.node_states.keys():
			self.node_states[key][2][-1] = self.max_time - self.node_states[key][1][-1]

		for key in reversed(self.node_states.keys()): 
			colors = [ color_dict[i] for i in self.node_states[key][0] ]

			self.get_logger().info(f'self.node_states: {self.node_states[key][0]}')
			self.get_logger().info(f'self.node_states: {self.node_states[key][1]}')
			self.get_logger().info(f'self.node_states: {self.node_states[key][2]}')
			self.get_logger().info(f'colors: {colors}')


			ax.barh(y=height, width=self.node_states[key][2], height=height_iter, left=self.node_states[key][1], color=colors)

			# [ ax.barh(y=height, width=self.node_states[key][2][i], height=height_iter, left=self.node_states[key][1][i], color=colors[i]) for i in range(len(colors)) ]

			height += height_iter + .01

		ax.set_yticks( [ .05 + i * ( height_iter + .01 ) for i in range(len(self.node_states.keys())) ], labels=reversed(self.node_states.keys()))
		ax.set_xlabel('Time')
		ax.set_title('Node status over time')
		ax.legend()

		plt.tight_layout()
		plt.show()

def main():
	rclpy.init()

	srv = ServerNode()

	srv.record_results()
	srv.graph_results()

if __name__ == '__main__':
	main()