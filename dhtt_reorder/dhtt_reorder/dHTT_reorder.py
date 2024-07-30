import rclpy
import rclpy.logging
import rclpy.node

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest, NutreeJsonRequest
from dhtt_msgs.msg import Subtree as dHTTSubtree, Node as dHTTNode

from dhtt_plot.build_nutree import dHTTHelpers, HashabledHTTNode

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

    def isBehaviorNode(self, node: NutreeNode) -> bool:
        if self._usingdHTT:
            return dHTTHelpers.isBehaviorNode(node)
        else:
            return node.name not in HTT.TASKNODES

    def constructThenNode(self):
        """Construct a HashabledHTTNode THEN node with only the name, type and plugin fields set

        Other fields remain defaults from dHTTNode constructor
        """
        if self._usingdHTT:
            return HashabledHTTNode(dHTTNode(node_name="THEN", type=dHTTNode.THEN, plugin_name='dhtt_plugins::ThenBehavior'))
        else:
            return "THEN"

    def constructAndNode(self):
        """Construct a HashabledHTTNode AND node with only the name, type and plugin fields set

        Other fields remain defaults from dHTTNode constructor
        """
        if self._usingdHTT:
            return HashabledHTTNode(dHTTNode(node_name="AND", type=dHTTNode.AND, plugin_name='dhtt_plugins::AndBehavior'))
        else:
            return "AND"

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

                # remove the root node
                assert self.tree.first_child().data.node_name == dHTTHelpers.ROOTNAME
                self.tree.first_child().remove(keep_children=True)

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
        # TODO needs to be AND node
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
            assert (x.is_leaf())
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
        if self._usingdHTT:
            return set([x.data.node_name for x in tree])
        else:
            return set([x.name for x in tree])

    def reorder(self, before: "list[NutreeNode]", after: "list[NutreeNode]", debug=False, autoPrune=True):
        scope: NutreeNode = self.minimumInclusiveScope(before + after)
        beforePrime: NutreeTree = self.primeSelect(
            include=before, exclude=after)
        afterPrime: NutreeTree = self.primeSelect(
            include=after, exclude=before)

        # we are deleting nodes, so if we want to refer to before/after again, we go by names
        if self._usingdHTT:
            beforeNames = [node.data.node_name for node in before]
            afterNames = [node.data.node_name for node in after]
        else:
            beforeNames = [node.name for node in before]
            afterNames = [node.name for node in after]

        beforePrimeBehaviors = [
            node for node in beforePrime if self.isBehaviorNode(node)]

        afterPrimeBehaviors = [
            node for node in afterPrime if self.isBehaviorNode(node)]
        # excludeBehaviors = beforePrimeBehaviors + afterPrimeBehaviors
        # includeBehaviors = [node for node in self.tree if node.name not in HTT.TASKNODES] - excludeBehaviors
        # subtractedTree = self.primeSelect(include=includeBehaviors, exclude=excludeBehaviors)
        # or just move/delete in place

        if debug:
            representation = "{node.data.node_name}" if self._usingdHTT else "{node.name}"
            print(
                "################################################################################")
            print(
                f'Tree before reorder {[x for x in beforeNames]} before {[x for x in afterNames]}:')
            self.tree.print(repr=representation)
            print("\nScope:\n", scope.format(repr=representation))

            print("\nbeforePrime:")
            for x in beforePrime:
                print(x.format(repr=representation), '\n')

            print("afterPrime:")
            for x in afterPrime:
                print(x.format(repr=representation), '\n')

        if self.isThenNode(self.tree.first_child()):
            topAND = self.tree.add(self.constructAndNode())
            newTHEN = topAND.add(self.constructThenNode())
            self.tree.first_child().move_to(topAND)
        else:
            newTHEN = self.tree.first_child().add(self.constructThenNode())
        newANDA = newTHEN.add(self.constructAndNode())
        newANDB = newTHEN.add(self.constructAndNode())

        # remove the old nodes. We have copies in before/afterPrime
        # ideally, we'd want to move these instead, but they are technically different trees so nutree isn't happy
        for x in [_ for _ in self.tree]:
            if isinstance(x.data, str):
                if x.name in [y.name for y in beforePrimeBehaviors] + [y.name for y in afterPrimeBehaviors]:
                    assert x.name not in HTT.TASKNODES
                    x.remove()
            elif isinstance(x.data, HashabledHTTNode):
                # see another comment for questioning abuse of __dict__
                if x.data.__dict__ in [y.data.__dict__ for y in beforePrimeBehaviors] + [y.data.__dict__ for y in afterPrimeBehaviors]:
                    assert self.isBehaviorNode(x)
                    x.remove()

        newANDA.add(beforePrime)
        newANDB.add(afterPrime)

        if autoPrune:
            self.prune()

        if debug:
            print(
                f'Tree after reorder {[x for x in beforeNames]} before {[x for x in afterNames]}:')
            print(self.tree.format(repr=representation), '\n')

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

        if self.isTaskNode(node):
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
                    for child in (x for x in node.children.copy() if x.data == node.data):
                        child.remove(keep_children=True)
                        nextRecurse = [node]

            for x in nextRecurse:
                self.prune(x, combine=combine)
        else:
            return

    def filteredHTT(self, nodes: "list[NutreeNode]") -> 'HTT':
        """Creates a new copy of a tree and filters on provided list of nodes and task nodes
        """

        # make a copy of the tree
        if self._usingdHTT:
            newHTT = HTT(withdHTT=True)
            f = StringIO()
            self.tree.save(f, mapper=dHTTHelpers.nutreeSerializeMapper)
            # the separate StringIO object is important
            fa = StringIO(f.getvalue())
            newHTT.tree = NutreeTree().load(fa, mapper=dHTTHelpers.nutreeDeserializerMapper)
        else:
            newHTT = HTT()
            newHTT.tree = NutreeTree.from_dict(self.tree.to_dict_list())
            newHTT.tree.filter(lambda node: node.name in [
                x.name for x in nodes] or node.name in HTT.TASKNODES)

        # TODO is this is a better case for defining an __eq__ function?
        def dHTTMatcher(node): return node.data.__dict__ in [
            x.data.__dict__ for x in nodes]

        def nondHTTMatcher(node): return node.data in [x.data for x in nodes]
        matcher = dHTTMatcher if self._usingdHTT else nondHTTMatcher

        # filter deletes anything that doesn't match this predicate.
        # However, it keeps their entire parent tree up to the root.
        # This means behaviors (leafs) and task nodes (internal nodes)
        newHTT.tree.filter(lambda node: matcher(node))
        newHTT.prune()
        return newHTT

    def primeSelect(self, include: "list[NutreeNode]", exclude: "list[NutreeNode]") -> NutreeTree:
        """From a set of nodes, selects the maximum exclusive scope then filters it

        Remember that filteredHTT() returns a new tree.
        """
        primeTree = self.maximumExclusiveScope(
            include=include, exclude=exclude)
        primeSet: list[NutreeNode] = []
        for tree in primeTree:
            for node in tree.iterator(add_self=True):
                if self.isBehaviorNode(node):
                    primeSet.append(node)

        if self._usingdHTT:
            assert len(set([node.data.node_name for node in primeSet]) &
                       set([node.data.node_name for node in exclude])) == 0
        else:
            assert len(set([node.name for node in primeSet]) &
                       set([node.name for node in exclude])) == 0

        return self.filteredHTT(primeSet).tree

    def diffMergedHTT(self, targetTree: NutreeTree):
        """After reordering, update the dHTT tree to match the reordered Nutree tree

        There are smarter ways to do this, but the easiest is to make a new tree of subtasks,
        and move the behavior nodes over. Task nodes are newly created and old ones removed,
        behavior nodes are reparented.
        """
        toRemove = targetTree.first_child().data.node_name
        self._diffMergeHelper(targetTree.first_child(),
                              dHTTHelpers.ROOTNAME)  # assume the nutree is rooted at ROOT_0

        # delete the old subtree
        rs: ModifyRequest.Response = self._deleteNode(toRemove)
        assert rs.success

    def _diffMergeHelper(self, target: NutreeNode, updatedParentName: dHTTNode.parent_name):
        """Recursively add task nodes to the new subtree, or reparent behavior nodes"""
        if self.isTaskNode(target):
            rs: ModifyRequest.Response = self._addNodeTodHTT(
                target.data, updatedParentName)
            assert rs.success
            newNodeName = rs.added_nodes[0]
            target.data.node_name = newNodeName

            # recurse
            for child in target.children:
                self._diffMergeHelper(child, newNodeName)

        elif self.isBehaviorNode(target):
            rs: ModifyRequest.Response = self._reparentNode(
                target.data.node_name, updatedParentName)
            assert rs.success

        # return nodes to delete

    def _addNodeTodHTT(self, targetNode: HashabledHTTNode, updatedParentName: dHTTNode.parent_name) -> ModifyRequest.Response:
        modifyRQ = ModifyRequest.Request()
        modifyRQ.type = ModifyRequest.Request.ADD
        modifyRQ.to_modify.append(updatedParentName)
        modifyRQ.add_node = dHTTNode()
        modifyRQ.add_node.type = targetNode.type
        modifyRQ.add_node.node_name = targetNode.node_name
        modifyRQ.add_node.plugin_name = targetNode.plugin_name

        future = self.node.modifyClient.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def _reparentNode(self, targetName: str, parentName: str) -> ModifyRequest.Response:
        modifyRQ = ModifyRequest.Request()
        modifyRQ.type = ModifyRequest.Request.REPARENT
        modifyRQ.to_modify = [targetName]
        modifyRQ.new_parent = parentName

        future = self.node.modifyClient.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def _deleteNode(self, targetName: dHTTNode.node_name) -> ModifyRequest.Response:
        modifyRQ = ModifyRequest.Request()
        modifyRQ.type = ModifyRequest.Request.REMOVE
        modifyRQ.to_modify.append(targetName)

        future = self.node.modifyClient.call_async(modifyRQ)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()


def main():
    print('Hi from dhtt_reorder.')


if __name__ == '__main__':
    main()
