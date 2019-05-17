import logging, unittest, collections
from difflib import SequenceMatcher

log = logging.getLogger(__name__)


class HomogenoeusComposable:
    '''
    A generic type for objects whose composition yields another object
    of the same type.

    Objects simply store a tuple with whatever data is passed to the
    constructor; composition is implemented by concatenating the data
    of the two instances.
    '''

    @staticmethod
    def composeAll(composables, pairWiseSwap=False) :
        if pairWiseSwap :
            other = composables[0]
            for c in composables[1:] :
                other = c.compose( [other] )
            return other
        else :
            return composables[0].compose( composables[1:] )

    def __init__(self, anItem):
        self.data = tuple(anItem)

    def compose(self, others):
        l = list(self.data)
        for other in others : l.extend( other.data )
        return HomogenoeusComposable( l )

    def __hash__(self): return 37*hash(self.data)
    def __eq__  (self, other): return self.data.__eq__(other.data)
    def __str__(self):
        return "«" + " ".join( [e.__str__() for e in self.data] ) + "»"


class BinaryComposition:
    def __init__(self, c1, c2):
        self.arg1 = c1
        self.arg2 = c2
        self.result = c1.compose([c2])

class Path:
    '''
    A modifiable sequence of composable objects.

    Any subsequence of the path can be composed "in place" into a single
    item, thus altering the original sequence.

    For example, the Path 'a b c d e f' can become 'a bcd e f' after
    composing the subsequence 'b c d'. In that case, the length of the Path
    would change from 6 to 4.
    '''

    SeqInfo = collections.namedtuple('SeqInfo', ['start', 'size'])

    def __init__(self, composablesList, pairWiseSwap=False):
        self.items = composablesList
        self.flags = [ False for _ in self.items]
        self.mySubPoses = []
        self.pairWiseSwap = pairWiseSwap

    def len(self): return len(self.items)

    def match(self, other):
        s = SequenceMatcher(None, self.items, other.items)
        return s.find_longest_match(0, self.len(), 0, other.len() )

    def composeSubPath(self, sequenceInfo):
        # Identify the slice of the path, and merge the corresponding elements:
        if sequenceInfo.size <= 0: return
        beg = sequenceInfo.start
        siz = sequenceInfo.size
        end = beg + siz
        composite = HomogenoeusComposable.composeAll(self.items[beg:end], self.pairWiseSwap)

        # Now delete the elements which have been merged, and replace them with
        # the new, single, composite item
        for _ in range(siz):
            del self.items[beg] # do not increase the index! The 'del' reduces the length!
            del self.flags[beg]
        self.items.insert(beg, composite)
        self.flags.insert(beg, True)

        # Propagate the shrinking event to the subpaths, which hold an index of
        # this path's items
        for sp in self.mySubPoses :
            if beg < sp.cBeg :
                sp.cBeg = sp.cBeg - (siz - 1)

    def uncomposedPaths(self):
        ret = []
        i = 0
        while i < len(self.flags) :
            while i<len(self.flags) and self.flags[i]     : i = i+1
            beg = i
            while i<len(self.flags) and not self.flags[i] : i = i+1
            end = i
            if (end-beg)>0 and end<=len(self.flags) :
                sp = SubPath(self, beg, end, self.pairWiseSwap)
                self.mySubPoses.append( sp )
                ret.append( sp )
        return ret

    def __str__(self):
        return " ".join( [item.__str__() for item in self.items] )
    def __repr__(self):
        return self.__str__()


class SubPath(Path):
    '''A Path object that is aware of being a subsequence of a container Path.

    Composition operations applied to an instance of SubPath are reflected into
    the container Path. The mechanism can be applied recursively.
    '''
    # At the moment, subpaths are not used in the routine to find the optimal
    # sequence of compositions

    def __init__(self, container, beg, end, pairWiseSwap=False):
        Path.__init__( self, container.items[beg:end], pairWiseSwap )
        self.container = container
        self.cBeg = beg

    def composeSubPath(self, sequenceInfo):
        Path.composeSubPath(self, sequenceInfo)
        seqInfo = Path.SeqInfo(sequenceInfo.start + self.cBeg, sequenceInfo.size)
        self.container.composeSubPath(seqInfo)

'''
A placeholder to keep track of a composition operation applied to some Path
instances.
'''
class Composition:
    class Involved:
        def __init__(self, path, seq):
            self.path    = path
            self.interval= seq
        def applyCompose(self):
            self.path.composeSubPath( self.interval )

    def __init__(self, path, seqInfo):
        self.involved    = [ Composition.Involved(path, seqInfo) ]
        self.composables = self._subSequence(path.items, seqInfo)
        self.path = Path(self.composables, path.pairWiseSwap)

    def shrink(self, seqInfo):
        for inv in self.involved :
            a = inv.interval.start + seqInfo.start
            inv.interval = Path.SeqInfo(a, seqInfo.size)
        self.composables = self._subSequence(self.composables, seqInfo)
        self.path = Path(self.composables, self.path.pairWiseSwap)

    def addInvolved(self, path, seqInfo):
        self.involved.append( Composition.Involved(path, seqInfo) )
        # ASSERT( len(self.composables) == seqInfo.size )

    def asPath(self): return self.path

    def _subSequence(self, sequence, info):
        beg = info.start
        end = beg + info.size
        return sequence[beg:end]

    def __str__(self):
        return self.asPath().__str__()
    def __repr__(self):
        return self.__str__()

    def asSequenceOfBinaryCompositions(self):
        if self.path.pairWiseSwap :
            bcf = lambda arg1, arg2: BinaryComposition(arg2,arg1) # note the swap
        else :
            bcf = lambda arg1, arg2: BinaryComposition(arg1,arg2)
        ret = []
        c0 = self.composables[0]
        for c in self.composables[1:]:
            bc = bcf(c0,c)
            c0 = bc.result
            ret.append( bc )
        return ret

'''
Find the longest sub-path (>1 item) common to the largest number of Paths.
The return value is a list of Composition obejcts, since a Composition is
essentially a sub-path.

Being shared by a higher number of paths is more important than being longer;
for example, assuming to use character sequences to represent paths, the result
of this function for the arguments 'bc', 'abc', 'bcdef', 'cdef' is ['bc','def'],
even though 'cdef' is a longer, common subsequence.
'''
def findComposes(paths):
    compositions = []
    start = 1
    # Compare each Path with all the following ones. Therefore, skip the last
    # item, and in the inner loop start from the subsequent item, not the
    # beginning
    for p in paths[:-1] :
        if p.len() > 1 :
            composition = Composition( p, Path.SeqInfo(0, p.len()) )
            for p2 in paths :
                if p2 != p :
                    overlap = composition.asPath().match( p2 )
                    if overlap.size > 1 :
                        # Refine (shrink) the shared path with the new intersection
                        composition.shrink( Path.SeqInfo(overlap.a, overlap.size) )
                        # Record that p2 is affected by the current composition
                        composition.addInvolved(p2, Path.SeqInfo(overlap.b, overlap.size))

            # Apply the composition in all the involved paths. If there is only one,
            # it means that the current path has no overlap with anyone else, and
            # the composition object refers to its entirety; therefore it would seem
            # correct to call the applyCompose() also in such a case, to compose the
            # path itself in one go. Unfortunately, that leads to undesired results
            # when the path is a subpath of a larger one (which is transparent in
            # this function); composing a "full" path when it has no more overlaps
            # must be done in the outer context with visibility of the complete,
            # actual paths of the use case.
            if len(composition.involved) > 1:
                for inv in composition.involved :
                    inv.applyCompose()
                compositions.append( composition )
        start = start+1

    return compositions

'''
Find the "optimal" (?!?) sequence of compositions that bring all the given Paths
(i.e. sequence of composables) to be an individual (composed) item.

The optimality refers to composing the items (and in turn the composites)
which are shared by the largest number of Paths, to avoid repeating the same
compositions multiple times.
'''
def allComposes(paths):
    opool = sorted( paths, key=Path.len ) # sort the input list based on the length
    pool = opool
    totComposes = []
#    iter = 0

    composes = findComposes( pool )
    while len(composes) > 0 :
#        iter = iter + 1
        totComposes.extend( composes )
#         print("\niter ", iter)
#         for p in composes:
#             print(p)
#         print("")
#         for p in pool :
#             print(p)

        composes = findComposes( pool )

    # No more overlaps, force the composition of whatever is left
    for p in opool :
        if p.len()>1 :
            wholeSpan = Path.SeqInfo(0, p.len())
            totComposes.append( Composition(p, wholeSpan) )
            p.composeSubPath( wholeSpan )

    return totComposes






class TestBase(unittest.TestCase):
    @staticmethod
    def stringToComposablesList(astring):
        return [HomogenoeusComposable(c) for c in astring]

    def __init__(self, *args, **kwargs):
        super(TestBase, self).__init__(*args, **kwargs)

        self.paths = [Path(TestBase.stringToComposablesList(s)) for s in self.inputSequences ]
        self.composes = allComposes(self.paths)

    def makeExpected(self, patternsList):
        self.expected = []
        for pattern in patternsList:
            self.expected.append( Path( [HomogenoeusComposable(stri) for stri in pattern] ) )

    # Do not name this method 'test_..' otherwise the unittest framework takes
    # it as an actual unit test and tries to instantiate it directly, which
    # fails (because its constructor is intended to be called only after the
    # constructor of the subclasses).
    def myCmpTest(self):
        idx = 0
        for p in self.composes :
            self.assertEqual(p.__str__(), self.expected[idx].__str__())
            idx = idx + 1


class Test0(TestBase):
    def __init__(self, *args, **kwargs):
        self.inputSequences = [ 'abcde', 'cdefg']
        super(Test0, self).__init__(*args, **kwargs)
        self.makeExpected([ ['c','d','e'] , ['a', 'b', 'cde'] , ['cde', 'f', 'g']] )

    def test_stuff(self):  self.myCmpTest()

class Test1(TestBase):
    def __init__(self, *args, **kwargs):
        self.inputSequences = [ 'abcdefghil', 'defgh', 'bc' ]
        super(Test1, self).__init__(*args, **kwargs)
        self.makeExpected( [
            ['b','c'],
            ['d','e','f','g','h'],
            ['a','bc','defgh','i','l'] ] )

    def test_stuff(self):  self.myCmpTest()

class Test2(TestBase):
    def __init__(self, *args, **kwargs):
        # TODO the sorting order of equally long sequences
        self.inputSequences = ['h', 'kifg', 'abfijl', 'kifbc', 'fij', 'cbfi']
        super(Test2, self).__init__(*args, **kwargs)

        # TODO equivalent alternatives!!!
        # bfi  fij : '«a» «b f i» «j» «l»' or '«a» «b» «f i j» «l»'
        self.makeExpected([
            ['f','i'], ['k','i','f'], ['b','fi'], ['fi','j'], ['kif','g'],
            ['c','bfi'], ['kif','b','c'], ['a','bfi','j','l'] ])

    def test_stuff(self):  self.myCmpTest()

class Test3(TestBase):
    def __init__(self, *args, **kwargs):
        self.inputSequences = ['abcdefghi', 'bc', 'fg', 'bcdefghil']
        super(Test3, self).__init__(*args, **kwargs)

        self.makeExpected([
            ['b','c'], ['f','g'], ['bc','d','e','fg','h','i'],
            ['a','bcdefghi'], ['bcdefghi','l'] ])

    def test_stuff(self):  self.myCmpTest()



class Test4(TestBase):
    '''This is the first test that requires two iterations of the loop in
    allComposes()
    Note that again, multiple solutions are possible: different compositions
    as well as a different order of the same compositions. Here I set the
    expected such that the test passes; it is not cheating, but it is not
    robust.
    '''
    def __init__(self, *args, **kwargs):
        self.inputSequences = ['fghi', 'hijkl', 'fghijk', 'fghijkl']
        super(Test4, self).__init__(*args, **kwargs)

        self.makeExpected([
            ['h','i'], ['hi','j','k'], ['f','g'],
            ['hijk','l'], ['fg','hi'], ['fg', 'hijk'], ['fg','hijkl'] ])

    def test_stuff(self):  self.myCmpTest()



class Test5(TestBase):
    def __init__(self, *args, **kwargs):
        self.inputSequences = ['abcdefghijkl', 'fghijkl', 'hijkl', 'a', 'abc','abcde','abcdefg','abcdefghi','abcdefghijk']
        super().__init__(*args, **kwargs)

        self.makeExpected([
            ['a','b','c'], ['h','i'],['abc','d','e'],['hi','j','k'],
            ['f','g'],['abcde','fg'],['abcdefg','hijk'],
            ['hijk','l'],['fg','hijkl'],['abcdefg','hi'],['abcdefghijk', 'l']
            ])

    def test_stuff(self):  self.myCmpTest()

class TestManual(TestBase):
    def __init__(self, *args, **kwargs):
        self.inputSequences = ['fghi', 'hijk', 'efg', 'defg', 'cdefg']
        super().__init__(*args, **kwargs)


def manual():
    test = TestManual()
    print("")
    for p in test.composes:
        print(p)


if __name__ == "__main__" :
    unittest.main()
    #manual()



