import filecmp
import difflib
import pathlib
import re
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

def nextLine(istream):
    n = 1
    line = istream.readline()

    checkBlank   = re.compile('^ *$')
    checkComment = re.compile('^ *--')
    while line != '':
        if not (checkBlank.match(line) or checkComment.match(line)):
            yield n,line
        line = istream.readline()
        n = n + 1


dir_output   = pathlib.Path("output")
dir_expected = pathlib.Path("expected_output")

dir_comparator = filecmp.dircmp(dir_output, dir_expected)

if len(dir_comparator.right_only) > 0 :
    raise RuntimeError("some expected files are missing")

if len(dir_comparator.left_only) > 0 :
    raise RuntimeError("unexpected generated files")

for file in dir_comparator.common_files :
    # filecmp.cmp(dir_expected/file, dir_output/file) # cant use it, because it does not have exclusion patterns
    with open(dir_expected / file) as reference:
        with open(dir_output / file) as generated:
            for (rn,rline), (gn, gline) in zip(nextLine(reference), nextLine(generated)):
                if rline != gline:
                    logger.error("mismatch for file '%s'", file)
                    logger.error("  EXPECTED (line %d): %s", rn, rline)
                    logger.error("  ACTUAL   (line %d): %s", gn, gline)

