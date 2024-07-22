from nutree import Tree, Node, IterMethod
from sympy.utilities.iterables import multiset_permutations


class HTT:
    TASKNODES = {"AND", "THEN", "OR"}

    def __init__(self):
        self.tree = Tree("testHTT")

    def createTree(self, children: set):
        self.tree.add("AND")
        for child in children:
            self.tree.first_child().add(child)

    def resetTree(self):
        self.tree.clear()
        self.createTree()

    def minimumInclusiveScope(self, nodes: "list[Node]") -> Node:
        minNode = self.findShallowestNode(nodes)

        assert (minNode != None)

        currentParent: Node = minNode.parent
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

    def findShallowestNode(self, nodes: "list[Node]"):
        minNode: Node = None
        for x in nodes:
            assert (x.is_leaf)
            if minNode == None or x.depth() < minNode.depth():
                minNode = x
        return minNode

    def maximumExclusiveScope(self, include: "list[Node]", exclude: "list[Node]") -> "list[Node]":
        maxList = include

        minNode = self.findShallowestNode(include)

        currentParent: Node = minNode.parent
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

    def setOfAllChildren(self, tree: Tree) -> set:
        return set([x.name for x in tree])

    def reorder(self, before: "list[Node]", after: "list[Node]", debug=False, autoPrune=True):
        scope: Node = self.minimumInclusiveScope(before + after)
        beforePrime: Tree = self.primeSelect(
            include=before, exclude=after)
        afterPrime: Tree = self.primeSelect(
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
            assert self.checkOrder(before, after)
            print(
                f'Tree after reorder {[x.name for x in before]} before {[x.name for x in after]}:')
            print(self.tree.format(), '\n')

    def prune(self, node: "Node" = None, combine: bool = False):
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
            nextRecurse: list[Node] = node.children

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

    def filteredHTT(self, nodes: "list[Node]") -> 'HTT':
        newHTT = HTT()
        newHTT.tree = Tree.from_dict(self.tree.to_dict_list())
        newHTT.tree.filter(lambda node: node.name in [
            x.name for x in nodes] or node.name in HTT.TASKNODES)
        newHTT.prune()
        return newHTT

    def primeSelect(self, include: "list[Node]", exclude: "list[Node]") -> Tree:
        primeTree = self.maximumExclusiveScope(
            include=include, exclude=exclude)
        primeSet: list[Node] = []
        for tree in primeTree:
            for node in tree.iterator(add_self=True):
                if node.name not in HTT.TASKNODES:
                    primeSet.append(node)
        assert len(set([node.name for node in primeSet]) &
                   set([node.name for node in exclude])) == 0

        return self.filteredHTT(primeSet).tree

    def checkOrder(self, before: 'list[Node]', after: 'list[Node]'):
        ordered = self.checkOrderHelper(self.tree.first_child(), before, after)
        latestBefore = max([ordered.index(node)
                            for node in before if node in ordered], default=None)
        earliestAfter = min([ordered.index(node)
                            for node in after if node in ordered], default=None)

        if latestBefore and earliestAfter:
            if latestBefore < earliestAfter:
                return True
            else:
                return False
        else:
            return True

    def checkOrderHelper(self, tree: Node, before: 'list[Node]', after: 'list[Node]') -> 'list[Node]':
        """
        base case: node is leaf, return node
        recurse: node is THEN, recurse on children in order, append results
        recurse: node is AND, recurse on children in any order, append results
        """
        ordered: 'list[Node]' = []

        if tree.has_children == False:
            return [tree]
        elif tree.name == 'THEN':
            for childIndex in range(len(tree.children)):  # in order
                ret = self.checkOrderHelper(
                    tree.children[childIndex], before, after)
                for node in ret:
                    ordered.append(node)
        elif tree.name == 'AND':
            for child in tree.children:
                ret = self.checkOrderHelper(child, before, after)
                for node in ret:
                    ordered.append(node)

        return ordered

def main():
    print('Hi from dhtt_reorder.')


if __name__ == '__main__':
    main()
