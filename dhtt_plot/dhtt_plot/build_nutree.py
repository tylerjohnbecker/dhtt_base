import rclpy
import rclpy.logging
import rclpy.node

from dhtt_msgs.srv import ModifyRequest, FetchRequest, NutreeJsonRequest
from dhtt_msgs.msg import Subtree as dHTTSubtree, Node as dHTTNode

from nutree import Tree as nutreeTree, Node as nutreeNode, IterMethod
# TODO capitalize nutreeTree...

from io import StringIO
import jsonpickle

# TODO rosdep python packages https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html

"""Package for turning a dHTT into a Nutree.

Nutree is helpful as a library for tree operations and has features 
for exporting to graphViz and other fomats.

dhtt_reorder package uses Nutree as an intermediate form to perform 
reordering operations before sending commands to the dhtt server.
"""


class HashabledHTTNode:
    """Provides a struct with non-hashable dHTTNode fields stripped
    """

    def __init__(self, node: dHTTNode):
        # head
        self.node_name = node.node_name
        self.plugin_name = node.plugin_name
        self.parent = node.parent
        self.parent_name = node.parent_name
        self.children = tuple(node.children)
        self.child_name = tuple(node.child_name)
        self.params = tuple(node.params)
        self.type = node.type
        self.plugin_name = node.plugin_name
        self.owned_resources = tuple(node.owned_resources)
        # node_status

    def __str__(self) -> str:
        return self.toJson()

    def toJson(self):
        return jsonpickle.encode(self)

    def todHTTNode(self) -> dHTTNode:
        ret = dHTTNode()
        # head
        ret.node_name = self.node_name
        ret.plugin_name = self.plugin_name
        ret.parent = self.parent
        ret.parent_name = self.parent_name
        ret.children = list(self.children)
        ret.child_name = list(self.child_name)
        ret.params = list(self.params)
        ret.type = self.type
        ret.owned_resources = list(self.owned_resources)
        # node_status
        return ret


class dHTTHelpers():
    ROOTNAME = 'ROOT_0'
    TASKNODES = {dHTTNode.AND, dHTTNode.OR, dHTTNode.THEN}
    TASKNODEPLUGINS = {"dhtt_plugins::AndBehavior",
                       "dhtt_plugins::OrBehavior", "dhtt_plugins::ThenBehavior"}
    TASKNODESTOPLUGINS = {dHTTNode.AND: "dhtt_plugins::AndBehavior",
                          dHTTNode.OR: "dhtt_plugins::OrBehavior", dHTTNode.THEN: "dhtt_plugins::ThenBehavior"}

    def isTaskNode(node: 'dHTTNode|nutreeNode') -> bool:
        nodeType: str = None
        if isinstance(node, dHTTNode):
            nodeType = node.type
        elif isinstance(node, nutreeNode):
            nodeType = node.data.type
        return nodeType in dHTTHelpers.TASKNODES

    def isUnorderedNode(node: 'dHTTNode|nutreeNode') -> bool:
        nodeType: str = None
        if isinstance(node, dHTTNode):
            nodeType = node.type
        elif isinstance(node, nutreeNode):
            nodeType = node.data.type
        return nodeType in {dHTTNode.AND}

    def isOrderedNode(node: 'dHTTNode|nutreeNode') -> bool:
        nodeType: str = None
        if isinstance(node, dHTTNode):
            nodeType = node.type
        elif isinstance(node, nutreeNode):
            nodeType = node.data.type
        return nodeType in {dHTTNode.THEN}

    def isBehaviorNode(node: 'dHTTNode|nutreeNode') -> bool:
        nodeType: str = None
        if isinstance(node, dHTTNode):
            nodeType = node.type
        elif isinstance(node, nutreeNode):
            nodeType = node.data.type
        return nodeType == dHTTNode.BEHAVIOR

    def nutreeSerializeMapper(node: nutreeNode, data) -> dict:
        # we are using a custom object, so data comes in empty
        return {'str': str(node.data)}  # this should result in a jsonpickle

    def nutreeDeserializerMapper(parentNode: nutreeNode, data: dict) -> HashabledHTTNode:
        return jsonpickle.decode(data['str'])


class Colors:
    BLUE = "#648fff"
    PURPLE = "#785ef0"
    MAGENTA = "#dc267f"
    ORANGE = "#fe6100"
    YELLOW = "#ffb000"
    BLACK = "#000000"
    WHITE = "#ffffff"


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

        # type dHTTNode is not hashable we use a restricted type hashabledHTTNode instead
        # all we care about is node name and node type in this case, we can add other fields
        # if necessary
        #
        # see nutreeTree:calc_data_id(tree, data), where tree is a self, we don't
        # need that here
        tree = nutreeTree()  # expect all nodes to be type hashabledHTTNode
        nodes: 'list[HashabledHTTNode]' = []
        addedNodes: 'list[HashabledHTTNode]' = []
        for subtree in subtrees:
            for node in subtree.tree_nodes:
                nodes.append(HashabledHTTNode(node))

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
                    nodeToAddParent = next(
                        x for x in addedNodes if x.node_name == node.parent_name)
                    tree[nodeToAddParent].add(nodeToAdd)
                    addedNodes.append(nodeToAdd)

                elif node.node_name == dHTTHelpers.ROOTNAME:
                    nodeToAdd = notAddedNodes.pop(notAddedNodes.index(node))
                    tree.add(nodeToAdd)
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

    def getPrintableTree(self, client: NutreeClient, printRoot=False, split=True) -> 'tuple[nutreeTree, nutreeTree]':
        """Get a pretty printable version of a tree of HashabledHTTNodes

        Returns tuple (printableTree, unmodifiedTree)

        The default string representation of a HashabledHTTNode is unreadable and the
        graph plotters can't handle it. All we care about is the name, so replace
        node.data with that
        """
        # note that this assumes node names do not have an underscore.
        subtrees = client.getTree().found_subtrees
        referenceTree = client.buildNutreeFromSubtrees(subtrees)
        tree = client.buildNutreeFromSubtrees(subtrees)

        if not printRoot and tree.first_child().data.node_name == dHTTHelpers.ROOTNAME:
            tree.first_child().remove(keep_children=True)

        # TODO fix this for otherwise ambiguous names
        for node in tree:
            name = node.data.node_name.split(
                '_')[0] if split else node.data.node_name
            node.set_data(name)

        return (tree, referenceTree)

    # service callbacks
    def saveNutreeMermaid(self, client: NutreeClient):
        tree, _ = self.getPrintableTree(client)
        tree.to_mermaid_flowchart("mermaid.md", add_root=False, unique_nodes=False)
        # tree.to_mermaid_flowchart("mermaid.png", format="png", add_root=False)
        self.get_logger().info("Saved mermaid.md")

    # TODO handle node.data.node_name
    def drawNutreeDot(self, client: NutreeClient, redundantThens=False, filename="graph"):
        tree, referenceTree = self.getPrintableTree(client)

        def node_mapper(node: nutreeNode, attr_def: dict):
            # printable node has no type information, so find it in the reference tree
            nodeActual = next(x for x in referenceTree if x.data.node_name.split(
                '_')[0] == node.name.split('_')[0])  # split without _ just returns the string
            if dHTTHelpers.isTaskNode(nodeActual):
                attr_def["style"] = "solid,filled,rounded"  # rounded rectangle
                attr_def["fontname"] = "liberation sans bold"
            elif dHTTHelpers.isBehaviorNode(nodeActual):
                attr_def["margin"] = "0.055"

            if redundantThens and dHTTHelpers.isOrderedNode(nodeActual) and dHTTHelpers.isOrderedNode(nodeActual.parent):
                # colors then nodes that are children of then nodes magenta and dashed
                attr_def["style"] = "solid,dashed,rounded"
                attr_def["color"] = Colors.MAGENTA

        node_attrs = {"style": "solid,filled", "shape": "rectangle", "width": 0.5, "height": 0.5, "margin": "0.11",
                      "fillcolor": Colors.WHITE, "color": Colors.PURPLE, "penwidth": 2.8, "fontname": "liberation sans"}

        edge_attrs = {"dir": "none", "color": Colors.PURPLE}

        tree.to_dotfile(filename + ".gv", add_root=False, node_attrs=node_attrs,
                        edge_attrs=edge_attrs, node_mapper=node_mapper, unique_nodes=False)
        tree.to_dotfile(filename + ".png", format="png", add_root=False, node_attrs=node_attrs,
                        edge_attrs=edge_attrs, node_mapper=node_mapper, unique_nodes=False, graph_attrs={"dpi": 600})

        # TODO svg

        self.get_logger().info("Saved graph.gv and graph.png")

    def messageNutreeDot(self, client: NutreeClient):
        pass

    def nutreeJsonToString(self, client: NutreeClient) -> str:
        """Wrangles Nutree json output into a string for a ros service response
        """
        subtrees = client.getTree().found_subtrees
        tree = client.buildNutreeFromSubtrees(subtrees)
        f = StringIO()
        tree.save(f, mapper=dHTTHelpers.nutreeSerializeMapper)
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


def entryMermaid(args=None):
    rclpy.init(args=args)
    server = NutreeServer()
    server.saveNutreeMermaid(server.nutreeClient)
    server.nutreeClient.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


def entryDot(args=None):
    rclpy.init(args=args)
    server = NutreeServer()
    server.drawNutreeDot(server.nutreeClient)
    server.nutreeClient.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
