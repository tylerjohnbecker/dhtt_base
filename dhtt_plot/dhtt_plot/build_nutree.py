import rclpy
import rclpy.logging
import rclpy.node
import pathlib
import yaml

from dhtt_msgs.srv import ModifyRequest, FetchRequest, NutreeJsonRequest
from dhtt_msgs.msg import Subtree as dHTTSubtree, Node as dHTTNode

from nutree import Tree as nutreeTree, Node as nutreeNode, IterMethod

from io import StringIO

"""Package for turning a dHTT into a Nutree.

Nutree is helpful as a library for tree operations and has features 
for exporting to graphViz and other fomats.

dhtt_reorder package uses Nutree as an intermediate form to perform 
reordering operations before sending commands to the dhtt server.
"""


class dHTTHelpers():
    ROOTNAME = 'ROOT_0'
    TASKNODES = {dHTTNode.AND, dHTTNode.OR, dHTTNode.THEN}

    def isTaskNode(self, node: dHTTNode):
        # regex check
        pass


class NutreeClient(rclpy.node.Node):
    """ Handles dhtt->dhtt_plot client functions. Mostly getting the tree and building a Nutree.
    """

    def __init__(self):
        super().__init__('dHTT_plot_BuildNutree_client')

        # Service clients
        self.modifyClient = self.create_client(
            ModifyRequest, '/modify_service')
        assert self.modifyClient.wait_for_service(timeout_sec=1.0)

        self.fetchClient = self.create_client(FetchRequest, '/fetch_service')
        assert self.fetchClient.wait_for_service(timeout_sec=1.0)

        self.get_logger().info("Started NutreeClient")

    def getTree(self) -> FetchRequest.Response:
        fetchRQ: FetchRequest.Request = FetchRequest.Request()
        fetchRQ.return_full_subtree = True

        fetchFuture = self.fetchClient.call_async(fetchRQ)
        rclpy.spin_until_future_complete(self, fetchFuture)

        fetchRS: FetchRequest.Response = fetchFuture.result()

        if fetchRS.success:
            self.get_logger().info("Got subtree from dHTT server")
            self.get_logger().debug(f'Subtrees: {str(fetchRS.found_subtrees)}')
        else:
            self.get_logger().error(
                f'Failed to get subtree from dHTT server: {fetchRS.error_msg}')
        return fetchRS

    def buildNutreeFromSubtrees(self, subtrees: 'list[dHTTSubtree]') -> nutreeTree:
        """Takes a list of subtrees from a FetchRequest and produces a Nutree Tree

        Intended to follow from a FetchRequest.Response.found_subtrees
        """
        tree = nutreeTree()
        nodes: 'list[dHTTNode]' = []
        addedNodes: 'list[dHTTNode]' = []
        for subtree in subtrees:
            for node in subtree.tree_nodes:
                nodes.append(node)

        rootIndex = [i for i in range(
            len(nodes)) if nodes[i].node_name == dHTTHelpers.ROOTNAME]
        assert len(rootIndex) == 1, "no dHTT root node"

        # TODO assert no duplicates

        # we wrangle this a little bit because the parent pointer in Node.msg
        # is a string of the parent name
        notAddedNodes = [x for x in nodes]
        while len(notAddedNodes) > 0:
            for node in [x for x in notAddedNodes]:

                if node.parent_name in [x.node_name for x in addedNodes]:
                    nodeToAdd = notAddedNodes.pop(notAddedNodes.index(node))
                    tree[nodeToAdd.parent_name].add(nodeToAdd.node_name).set_meta(
                        'type', nodeToAdd.plugin_name)
                    addedNodes.append(nodeToAdd)
                elif node.node_name == dHTTHelpers.ROOTNAME:
                    nodeToAdd = notAddedNodes.pop(notAddedNodes.index(node))
                    tree.add(nodeToAdd.node_name).set_meta(
                        'type', nodeToAdd.plugin_name)
                    addedNodes.append(nodeToAdd)

        self.get_logger().info("Built Nutree tree")
        self.get_logger().debug(tree.format())
        return tree


class NutreeServer(rclpy.node.Node):
    """Handles dhtt->dhtt_plot server functions, like serializing to a json or outputting to a file.
    """

    def __init__(self):
        super().__init__('dHTT_plot_BuildNutree_server')

        self.nutreeClient = NutreeClient()

        # Service servers
        self.serializerServer = self.create_service(
            NutreeJsonRequest, '/nutree_json_service', self.nutree_json_service_callback)

        self.get_logger().info("Started NutreeServer")

    # service callbacks
    def saveNutreeMermaid(self, client: NutreeClient):
        subtrees = client.getTree().found_subtrees
        tree = client.buildNutreeFromSubtrees(subtrees)
        tree.to_mermaid_flowchart("mermaid.md", add_root=False)
        # tree.to_mermaid_flowchart("mermaid.png", format="png", add_root=False)
        self.get_logger().info("Saved mermaid.md")

    def drawNutreeDot(self, client: NutreeClient):
        subtrees = client.getTree().found_subtrees
        tree = client.buildNutreeFromSubtrees(subtrees)
        tree.to_dotfile("graph.gv", add_root=False)
        tree.to_dotfile("graph.png", format="png", add_root=False)
        self.get_logger().info("Saved graph.gv and graph.png")

    def messageNutreeDot(self, client: NutreeClient):
        pass

    def nutreeJsonToString(self, client: NutreeClient) -> str:
        """Wrangles Nutree json output into a string for a ros service response
        """
        subtrees = client.getTree().found_subtrees
        tree = client.buildNutreeFromSubtrees(subtrees)
        f = StringIO()
        tree.save(f)
        self.get_logger().info("Produced Nutree json")
        self.get_logger().debug(str(f.getvalue()))
        return f.getvalue()

    def nutree_json_service_callback(self, _, response: NutreeJsonRequest.Response):
        response.json = self.nutreeJsonToString(self.nutreeClient)
        return response


def main(args=None):
    rclpy.init(args=args)

    server = NutreeServer()
    rclpy.spin(server)

    server.nutreeClient.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
