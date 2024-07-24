import rclpy
import rclpy.logging
import rclpy.node

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, NutreeJsonRequest
from dhtt_msgs.msg import Subtree as dHTTSubtree, Node as dHTTNode

from dhtt_plot.build_nutree import dHTTHelpers

from nutree import Tree as NutreeTree, Node as NutreeNode, IterMethod

from io import StringIO


class Reorderer(rclpy.node.Node):
    def __init__(self):
        super().__init__('dHTT_reorder')

        # Service clients
        self.modifyClient = self.create_client(
            ModifyRequest, '/modify_service')
        assert self.modifyClient.wait_for_service(
            timeout_sec=1.0), "Did you forget?: ros2 run dhtt start_server"

        self.fetchClient = self.create_client(FetchRequest, '/fetch_service')
        assert self.fetchClient.wait_for_service(timeout_sec=1.0)

        self.controlClient = self.create_client(
            ControlRequest, '/control_service')
        assert self.controlClient.wait_for_service(timeout_sec=1.0)

        self.nutreeServerClient = self.create_client(
            NutreeJsonRequest, '/nutree_json_service')
        assert self.nutreeServerClient.wait_for_service(
            timeout_sec=1.0), "Did you forget?: ros2 run dhtt_plot build_nutree_server"

        self.get_logger().info("Started Reorderer")


class HTT:
    """Helper for managing internal HTT representation and mirroring changes on dHTT server

    Instantiate with withdHTT=True to spin up the client node and make changes on the server.
        Think of this as a dry run. Although members like setTreeFromHTT() will not work.
    """

    # legacy test support, used only when not running dHTT server
    TASKNODES = {"AND", "THEN", "OR"}

    def __init__(self, withdHTT=False):
        self.tree = NutreeTree("testHTT")
        self._usingdHTT = False

        if withdHTT:
            self.node = Reorderer()
            self._usingdHTT = True
        else:
            self.node = None
            print("Running without dHTT server")

    def isTaskNode(self, node: NutreeNode) -> bool:
        if self._usingdHTT:
            return dHTTHelpers.isTaskNode(node)
        else:
            return node.name in HTT.TASKNODES

    def isThenNode(self, node: NutreeNode) -> bool:
        if self._usingdHTT:
            return dHTTHelpers.isOrderedNode(node)
        else:
            return node.name == "THEN"

    def isAndNode(self, node: NutreeNode) -> bool:
        if self._usingdHTT:
            return dHTTHelpers.isUnorderedNode(node)
        else:
            return node.name == "AND"

    def setTreeFromdHTT(self) -> bool:
        if self._usingdHTT:
            rq = NutreeJsonRequest.Request()
            future = self.node.nutreeServerClient.call_async(rq)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=1.0)

            rs: NutreeJsonRequest.Response = future.result()
            if rs:
                self.tree = NutreeTree().load(
                    StringIO(rs.json), mapper=dHTTHelpers.nutreeDeserializerMapper)

                self.node.get_logger().info("Set tree from dhtt_plot")
                self.node.get_logger().debug(self.tree.format)
                return True
            else:
                self.node.get_logger().error("Did not get tree from dhtt_plot")
                return False
        else:
            print("usingdHTT is False, cannot set tree from dHTT")
            return False

    def createTree(self, children: set):
        """Creates a tree for reordering operations, top level and and a set of children.

        Takes a set of children, intended to be behavior nodes, and adds them to the Nutree and dHTT server"""
        self.tree.add("AND")
        for child in children:
            self.tree.first_child().add(child)

        # TODO add nodes to tree server

    def resetTree(self):
        self.tree.clear()
        self.createTree()

        # TODO reset tree server

    def minimumInclusiveScope(self, nodes: "list[NutreeNode]") -> NutreeNode:
        minNode = self.findShallowestNode(nodes)

        assert (minNode != None)

        currentParent: NutreeNode = minNode.parent
        foundCommonAncestor = False
        while currentParent != None and foundCommonAncestor == False:
            for x in nodes:
                if currentParent.is_ancestor_of(x) == False:
                    foundCommonAncestor = False
                    currentParent = currentParent.parent
                    break
                else:
                    foundCommonAncestor = True

        assert currentParent != None
        return currentParent

    def findShallowestNode(self, nodes: "list[NutreeNode]"):
        minNode: NutreeNode = None
        for x in nodes:
            assert (x.is_leaf)
            if minNode == None or x.depth() < minNode.depth():
                minNode = x
        return minNode

    def maximumExclusiveScope(self, include: "list[NutreeNode]", exclude: "list[NutreeNode]") -> "list[NutreeNode]":
        maxList = include

        minNode = self.findShallowestNode(include)

        currentParent: NutreeNode = minNode.parent
        while currentParent.parent != None:
            parentSet = self.setOfAllChildren(currentParent)
            includeSet = self.setOfAllChildren(include)
            excludeSet = self.setOfAllChildren(exclude)

            if parentSet.issuperset(includeSet) and parentSet.isdisjoint(excludeSet):
                maxList = [currentParent]
                currentParent = currentParent.parent
            else:
                break

        return maxList

    def setOfAllChildren(self, tree: NutreeTree) -> set:
        return set([x.name for x in tree])

    def reorder(self, before: "list[NutreeNode]", after: "list[NutreeNode]", debug=False, autoPrune=True):
        scope: NutreeNode = self.minimumInclusiveScope(before + after)
        beforePrime: NutreeTree = self.primeSelect(
            include=before, exclude=after)
        afterPrime: NutreeTree = self.primeSelect(
            include=after, exclude=before)

        # we are deleting nodes, so if we want to refer to before/after again, we go by names
        beforeNames = [node.name for node in before]
        afterNames = [node.name for node in after]

        beforePrimeBehaviors = [
            node for node in beforePrime if node.name not in HTT.TASKNODES]
        afterPrimeBehaviors = [
            node for node in afterPrime if node.name not in HTT.TASKNODES]
        # excludeBehaviors = beforePrimeBehaviors + afterPrimeBehaviors
        # includeBehaviors = [node for node in self.tree if node.name not in HTT.TASKNODES] - excludeBehaviors
        # subtractedTree = self.primeSelect(include=includeBehaviors, exclude=excludeBehaviors)
        # or just move/delete in place

        if debug:
            print(
                "################################################################################")
            print(
                f'Tree before reorder {[x.name for x in before]} before {[x.name for x in after]}:')
            self.tree.print()
            print("\nScope:\n", scope.format())

            print("\nbeforePrime:")
            for x in beforePrime:
                print(x.format(), '\n')

            print("afterPrime:")
            for x in afterPrime:
                print(x.format(), '\n')

        if self.tree.first_child().name == "THEN":
            topAND = self.tree.add("AND")
            newTHEN = topAND.add("THEN")
            self.tree.first_child().move_to(topAND)
        else:
            newTHEN = self.tree.first_child().add("THEN")
        newANDA = newTHEN.add("AND")
        newANDB = newTHEN.add("AND")

        # remove the old nodes. We have copies in before/afterPrime
        # ideally, we'd want to move these instead, but they are technically different trees so nutree isn't happy
        for x in [_ for _ in self.tree]:
            if x.name in [y.name for y in beforePrimeBehaviors] + [y.name for y in afterPrimeBehaviors]:
                assert x.name not in HTT.TASKNODES
                x.remove()

        newANDA.add(beforePrime)
        newANDB.add(afterPrime)

        if autoPrune:
            self.prune()

        if debug:
            before = [self.tree[nodeName] for nodeName in beforeNames]
            after = [self.tree[nodeName] for nodeName in afterNames]
            print(
                f'Tree after reorder {[x.name for x in before]} before {[x.name for x in after]}:')
            print(self.tree.format(), '\n')

    def prune(self, node: "NutreeNode" = None, combine: bool = False):
        """
        parameter combine: should be false always unless pretty printing the tree. It is essential 
        that the user's ordering is not altered (ie. reorder(A,B) and reorder(B,C) is not the same 
        as reorder(A, {B,C}))

        Base case: behavior node (behavior nodes -> leaves, but leaves -\> behavior node)
        Recurse: If we delete this node, recurse on the parent
        Recurse: If we promote a singleton child and/or have no children, recurse on parent
        Recurse: If we collapsed a like type child into us, recurse on ourself again
        Recurse: Otherwise, recurse on all task node children
        """
        # arguments can't refer to each other
        if node == None:
            node = self.tree.first_child()

        if node.name in HTT.TASKNODES:
            nextRecurse: list[NutreeNode] = node.children

            if len(node.children) <= 1:
                nextRecurse = [node.parent]
                node.remove(keep_children=True)

            # we may have just deleted it
            if node.data != "<deleted>":  # see tree.py _DELETED_TAG
                for child in (x for x in node.children.copy() if x.data == "<deleted>"):
                    print("RuRoh, found a registered but deleted node, deleting")
                    node.children.remove(child)
                    if not node.children:  # see Node::Remove()
                        node.children = None

                # combine like types
                if (combine):
                    for child in (x for x in node.children.copy() if x.name == node.name):
                        child.remove(keep_children=True)
                        nextRecurse = [node]

            for x in nextRecurse:
                self.prune(x, combine=combine)
        else:
            return

    def filteredHTT(self, nodes: "list[NutreeNode]") -> 'HTT':
        newHTT = HTT()
        newHTT.tree = NutreeTree.from_dict(self.tree.to_dict_list())
        newHTT.tree.filter(lambda node: node.name in [
            x.name for x in nodes] or node.name in HTT.TASKNODES)
        newHTT.prune()
        return newHTT

    def primeSelect(self, include: "list[NutreeNode]", exclude: "list[NutreeNode]") -> NutreeTree:
        primeTree = self.maximumExclusiveScope(
            include=include, exclude=exclude)
        primeSet: list[NutreeNode] = []
        for tree in primeTree:
            for node in tree.iterator(add_self=True):
                if node.name not in HTT.TASKNODES:
                    primeSet.append(node)
        assert len(set([node.name for node in primeSet]) &
                   set([node.name for node in exclude])) == 0

        return self.filteredHTT(primeSet).tree


def main():
    print('Hi from dhtt_reorder.')


if __name__ == '__main__':
    main()
