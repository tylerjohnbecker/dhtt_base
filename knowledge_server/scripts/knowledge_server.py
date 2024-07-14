#!/usr/bin/python3

import rclpy

from rclpy.node import Node

from dhtt_msgs.msg import Pair, UpdateInfo
from dhtt_msgs.srv import FetchInfo

class KnowledgeBase(Node):

	def __init__(self):
		super().__init__('knowledge_server_node')

		# knowledge starts simply as a blank dictionary
		self.knowledge_dict = {}

		self.update_sub = self.create_subscription(UpdateInfo, 'update_knowledge', self.update_callback, 10)
		self.fetch_srv = self.create_service(FetchInfo, 'fetch_knowledge', self.fetch_callback)

	def update_callback(self, data):
		self.get_logger().info('Knowledge update received!')
		for i in data.updates:
			self.knowledge_dict[i.key] = i.value

		self.get_logger().info(str(self.knowledge_dict))

	def fetch_callback(self, request, response):
		response.information_pairs = []

		for i in request.keys:
			n_pair = Pair()

			n_pair.key = i 

			if i in self.knowledge_dict.keys():
				n_pair.value = self.knowledge_dict[i]
			else:
				n_pair.value = ''

			response.information_pairs.append(n_pair)

		return response 


def main(args=None):
	rclpy.init(args=args)

	kb = KnowledgeBase()

	kb.get_logger().info('Server started...')

	rclpy.spin(kb)

	rclpy.shutdown()

if __name__ == '__main__':
	main()