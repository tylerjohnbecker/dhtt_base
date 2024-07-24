import pytest
import string
from nutree import Tree, Node
from dhtt_reorder.dHTT_reorder import HTT
import re


def setOfChildren(node: Node):
    return set([x.name for x in node.children])


def setOfAllChildren(tree: Tree):
    return set([x.name for x in tree])


class TestCreateHTT:
    originalChildren = set(string.ascii_lowercase)

    def test_init(self):
        htt = HTT()
        assert (isinstance(htt.tree, Tree))

    def test_createTree(self):
        htt = HTT()
        htt.createTree(set(self.originalChildren))
        assert (htt.tree.first_child() == "AND")
        assert (setOfChildren(htt.tree.first_child()) == self.originalChildren)

    def test_resetTree(self):
        htt = HTT()
        htt.createTree(set(self.originalChildren))
        htt.tree.first_child().add_child("Foo")
        assert (setOfChildren(htt.tree.first_child())
                == self.originalChildren.union({"Foo"}))

# class TestPermutations:
#     htt1 = HTT()


class TestReorderHTT:
    originalChildren = set(string.ascii_lowercase)

    def genOriginalHTT(self) -> HTT:
        htt = HTT()
        htt.createTree(self.originalChildren)
        return htt

    def test_minScope(self):
        htt = self.genOriginalHTT()
        before = [htt.tree['a']]
        after = [htt.tree['x']]
        assert (htt.minimumInclusiveScope(before + after) is htt.tree["AND"])

        htt.tree.first_child().add("THEN")
        htt.tree["THEN"].add("1")
        htt.tree["THEN"].add("2")
        before = [htt.tree['1']]
        after = [htt.tree['2']]
        assert (htt.minimumInclusiveScope(before + after) is htt.tree["THEN"])

        before = [htt.tree['a']]
        after = [htt.tree['2']]
        assert (htt.minimumInclusiveScope(before + after) is htt.tree["AND"])

    def test_maxScope(self):
        htt = self.genOriginalHTT()
        before = [htt.tree['a']]
        after = [htt.tree['x']]
        assert (htt.maximumExclusiveScope(
            include=before, exclude=after)[0] is htt.tree['a'])
        assert (len(htt.maximumExclusiveScope(
            include=before, exclude=after)) == 1)

        htt.tree.first_child().add("THEN")
        htt.tree["THEN"].add("1")
        htt.tree["THEN"].add("2")
        before = [htt.tree['1']]
        after = [htt.tree['2']]
        assert (htt.maximumExclusiveScope(
            include=before, exclude=after)[0] is htt.tree['1'])
        assert (len(htt.maximumExclusiveScope(
            include=before, exclude=after)) == 1)

        before = [htt.tree['1'], htt.tree['2']]
        after = [htt.tree['a']]
        assert (htt.maximumExclusiveScope(include=before,
                exclude=after)[0] is htt.tree['THEN'])
        assert (len(htt.maximumExclusiveScope(
            include=before, exclude=after)) == 1)

    def test_reorder(self):
        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after, debug=True)

        before = [htt.tree['b']]
        after = [htt.tree['c']]
        htt.reorder(before, after, debug=True)

        before = [htt.tree['c']]
        after = [htt.tree['a']]
        htt.reorder(before, after, debug=True)

        assert ','.join(x.name for x in htt.tree) in {
            'AND,d,THEN,c,THEN,a,b'}

    def test_prune(self):
        htt = HTT()
        AND1 = htt.tree.add("AND")
        AND2 = AND1.add("AND")
        AND2.add('a')
        AND2.add('b')

        htt.tree.print()
        htt.prune()
        htt.tree.print()

        assert ','.join(x.name for x in htt.tree) == 'AND,a,b'

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after, autoPrune=False)

        before = [htt.tree['b']]
        after = [htt.tree['c']]
        htt.reorder(before, after, autoPrune=False)

        before = [htt.tree['c']]
        after = [htt.tree['a']]
        htt.reorder(before, after, debug=True, autoPrune=False)
        htt.prune()
        htt.tree.print()

        assert ','.join(x.name for x in htt.tree) == 'AND,d,THEN,c,THEN,a,b'
        # changed from 'AND,d,THEN,c,a,b'

    def test_overrideEQ(self):
        # we need to override __eq__ otherwise list.remove() just does the first and node
        eqTree = Tree("foo")
        fooa = eqTree.add("foo")
        foob = eqTree.add("foo")
        assert fooa is not foob
        assert fooa != foob

    def test_reorderMultiple(self):
        """
        Have to eyeball this test too
        """
        htt = HTT()
        htt.createTree({'a', 'ax', 'ay', 'b', 'bx',
                       'by', 'c', 'cx', 'cy', 'd'})
        before = [htt.tree['a'], htt.tree['ax'], htt.tree['ay']]
        after = [htt.tree['b'], htt.tree['bx'], htt.tree['by']]
        htt.reorder(before, after, debug=True)

        before = [htt.tree['b'], htt.tree['bx'], htt.tree['by']]
        after = [htt.tree['c'], htt.tree['cx'], htt.tree['cy']]
        htt.reorder(before, after, debug=True)

        before = [htt.tree['cx'], htt.tree['ax']]
        after = [htt.tree['cy']]
        htt.reorder(before, after, debug=True)
        r = re.compile(
            'AND,d,THEN,THEN,AND,(a|ay),(a|ay),AND,(b|bx|by|c),(b|bx|by|c),(b|bx|by|c),(b|bx|by|c),THEN,THEN,ax,cx,cy')
        assert r.match(','.join(x.name for x in htt.tree))

    def test_reorder_repeat(self):
        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a'], htt.tree['c']]
        after = [htt.tree['b'], htt.tree['d']]
        htt.reorder(before, after)
        treeBefore = ','.join(x.name for x in htt.tree)
        before = [htt.tree['a'], htt.tree['c']]
        after = [htt.tree['b'], htt.tree['d']]
        htt.reorder(before, after, debug=True)
        treeAfter = ','.join(x.name for x in htt.tree)

        assert treeBefore == treeAfter

        before = [htt.tree['b'], htt.tree['d']]
        after = [htt.tree['c']]
        htt.reorder(before, after)
        treeBefore = ','.join(x.name for x in htt.tree)
        before = [htt.tree['b'], htt.tree['d']]
        after = [htt.tree['c']]
        htt.reorder(before, after, debug=True)
        treeAfter = ','.join(x.name for x in htt.tree)

        assert treeBefore == treeAfter

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['c']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after)
        treeBefore = ','.join(x.name for x in htt.tree)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        treeAfter = ','.join(x.name for x in htt.tree)

        assert treeBefore == treeAfter

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['c']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after)
        treeBefore = ','.join(x.name for x in htt.tree)
        treeBeforeAsHTT = HTT()
        treeBeforeAsHTT.tree = Tree.from_dict(htt.tree.to_dict_list())
        before = [htt.tree['a']]
        after = [htt.tree['b'], htt.tree['c'], htt.tree['d']]
        htt.reorder(before, after, debug=True)
        treeAfter = ','.join(x.name for x in htt.tree)

        assert treeBefore != treeAfter  # does not equal
        htt.prune(combine=True)
        treeBeforeAsHTT.prune(combine=True)
        treeBefore = ','.join(x.name for x in treeBeforeAsHTT.tree)
        treeAfter = ','.join(x.name for x in htt.tree)
        # now should equal, we gave b,c,d their own THEN, but its an equivalent tree when combined
        assert treeBefore == treeAfter

    def test_experiment4(self):
        """
        D and E are non-intersecting proper subsets of A and B and do
          not comprise the entire sets A and B

        D \subset A, D \subset B
        E \subset A, D \subset B
        D \cap E = \emptyset
        D \cup E \neq (A \cup B)

        X = {'a', 'a_d', 'a_e', 'b', 'b_d', 'b_e', 'c', '...'}
        D = {a_d, b_d}
        E = {a_e, b_e}

        Unfortunately, nutree appears to not be deterministic, so you have to eyeball this test
        """
        htt = HTT()
        htt.createTree({'a', 'ad', 'ae', 'b', 'bd', 'be', 'c', 'x'})
        before = [htt.tree['a'], htt.tree['ad'], htt.tree['ae']]
        after = [htt.tree['b'], htt.tree['bd'], htt.tree['be']]
        htt.reorder(before, after, debug=True)

        r = re.compile(
            'AND,(c|x),(c|x),THEN,AND,(a|ad|ae),(a|ad|ae),(a|ad|ae),AND,(b|bd|be),(b|bd|be),(b|bd|be)')
        assert r.match(','.join(x.name for x in htt.tree))

        before = [htt.tree['b'], htt.tree['bd'], htt.tree['be']]
        after = [htt.tree['c']]
        htt.reorder(before, after, debug=True)

        r = re.compile(
            'AND,x,THEN,THEN,AND,(a|ad|ae),(a|ad|ae),(a|ad|ae),AND,(b|bd|be),(b|bd|be),(b|bd|be),c')
        assert r.match(','.join(x.name for x in htt.tree))

        before = [htt.tree['c']]
        after = [htt.tree['a'], htt.tree['ad'], htt.tree['ae']]
        htt.reorder(before, after, debug=True)
        r = re.compile(
            'AND,x,THEN,c,THEN,AND,(a|ad|ae),(a|ad|ae),(a|ad|ae),AND,(b|bd|be),(b|bd|be),(b|bd|be)')
        assert r.match(','.join(x.name for x in htt.tree))

        before = [htt.tree['ad'], htt.tree['bd']]
        after = [htt.tree['ae'], htt.tree['be']]
        htt.reorder(before, after, debug=True)
        r = re.compile('AND,x,THEN,c,THEN,a,b,THEN,THEN,ad,bd,THEN,ae,be')
        assert r.match(','.join(x.name for x in htt.tree))

    def test_filteredTree(self):
        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['c']]
        htt.reorder(before, after)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)

        filtered = htt.filteredHTT(
            [htt.tree['b'], htt.tree['c'], htt.tree['d']])
        assert ','.join(x.name for x in filtered.tree) == 'THEN,THEN,b,c,d'
        filtered = htt.filteredHTT([htt.tree['a']])
        assert ','.join(x.name for x in filtered.tree) == 'a'

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b'], htt.tree['c']]
        htt.reorder(before, after)
        before = [htt.tree['a'], htt.tree['b'], htt.tree['c']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        pass

    def test_filter(self):
        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after, debug=True)
        before = [htt.tree['a']]
        after = [htt.tree['c']]
        htt.reorder(before, after, debug=True)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        htt.prune(combine=True)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        pass

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after, debug=True)
        before = [htt.tree['a']]
        after = [htt.tree['c']]
        htt.reorder(before, after, debug=True)
        before = [htt.tree['a']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        htt.prune(combine=True)
        before = [htt.tree['a']]
        after = [htt.tree['b'], htt.tree['c'], htt.tree['d']]
        htt.reorder(before, after, debug=True)
        pass

        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b'], htt.tree['c']]
        htt.reorder(before, after, debug=True)
        before = [htt.tree['a'], htt.tree['b'], htt.tree['c']]
        after = [htt.tree['d']]
        htt.reorder(before, after, debug=True)
        pass

    def test_helpers(self):
        htt = HTT()
        htt.createTree({'a', 'b', 'c', 'd'})
        before = [htt.tree['a']]
        after = [htt.tree['b']]
        htt.reorder(before, after)

        before = [htt.tree['b']]
        after = [htt.tree['c']]
        htt.reorder(before, after)

        before = [htt.tree['c']]
        after = [htt.tree['a']]
        htt.reorder(before, after)

        assert htt.isTaskNode(htt.tree.find_first('AND'))
        assert htt.isTaskNode(htt.tree.find_first('THEN'))
        assert htt.isTaskNode(htt.tree.find_first('a')) == False

        assert htt.isThenNode(htt.tree.find_first('THEN'))
        assert htt.isThenNode(htt.tree.find_first('AND')) == False
        assert htt.isThenNode(htt.tree.find_first('a')) == False

        assert htt.isAndNode(htt.tree.find_first('AND'))
        assert htt.isAndNode(htt.tree.find_first('THEN')) == False
        assert htt.isAndNode(htt.tree.find_first('a')) == False