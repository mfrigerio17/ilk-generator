import filecmp
import difflib
import pathlib
import logging

logger = logging.getLogger(__name__)


dir_output   = pathlib.Path("output")
dir_expected = pathlib.Path("expected_output")

dir_comparator = filecmp.dircmp(dir_output, dir_expected)

if len(dir_comparator.right_only) > 0 :
    raise RuntimeError("some expected files are missing")

if len(dir_comparator.left_only) > 0 :
    raise RuntimeError("unexpected generated files")

for file in dir_comparator.common_files :
    same = filecmp.cmp(dir_expected/file, dir_output/file)
    if not same:
        logger.error("File mismatch")
        with open(dir_expected / file) as reference:
            with open(dir_output / file) as generated:
                diff = difflib.unified_diff(reference.readlines(), generated.readlines(), "reference", "generated")
                for mismatch in diff :
                    print(mismatch)
                

#dir_comparator.report()
