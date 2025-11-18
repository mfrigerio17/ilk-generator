import unittest
import pathlib
import logging
import re
import filecmp

import rmt.rmt as rmtool

import ilkgenerator.query
import ilkgenerator.main

logger = logging.getLogger(__name__)

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




class TextComparisonTests(unittest.TestCase):
    def _run_generator(self, input_dir_path, output_dir_path):
        robot = input_dir_path / "model.kindsl"
        query = input_dir_path / "query.yaml"

        with open(query) as istream:
            userq = ilkgenerator.query.queryFromYAML( istream )

        _, robotmodel, robotframes, geometrymodel = rmtool.getmodels(str(robot))[0:4]

        qparser = ilkgenerator.query.QueryParser(robotmodel, robotframes, None)
        sweepingsolvers, iksolvers = qparser.validate(userq)

        ilkgenerator.main.generateILKFiles(output_dir_path, geometrymodel, sweepingsolvers, iksolvers)

    def _do_test_case(self, test_case_data_root):
        dir_inputs   = test_case_data_root / "inputs"
        dir_outputs  = test_case_data_root / "output"
        dir_expected = test_case_data_root / "expected_output"

        if not dir_outputs.exists():
            dir_outputs.mkdir(parents=True)
        self._run_generator(dir_inputs, dir_outputs)
        dir_comparator = filecmp.dircmp(dir_outputs, dir_expected)

        if len(dir_comparator.right_only) > 0 :
            logger.error("in %s: some expected files are missing", test_case_data_root)

        if len(dir_comparator.left_only) > 0 :
            logger.error("in %s: unexpected generated files", test_case_data_root)

        self.assertTrue( len(dir_comparator.right_only)==0 )
        self.assertTrue( len(dir_comparator.left_only) ==0 )

        for file in dir_comparator.common_files :
            # filecmp.cmp(dir_expected/file, dir_outputs/file) # cant use it, because it does not have exclusion patterns
            with open(dir_expected / file) as reference:
                with open(dir_outputs / file) as generated:
                    for (rn,rline), (gn, gline) in zip(nextLine(reference), nextLine(generated)):
                        sameLine = rline == gline
                        if not sameLine:
                            logger.error("in %s: mismatch for file '%s'", test_case_data_root, file)
                            logger.debug("  EXPECTED (line %d): %s", rn, rline)
                            logger.debug("  ACTUAL   (line %d): %s", gn, gline)
                        self.assertTrue( sameLine )


# Patch the test class, and dynamically add a test_.. method for each test case
# in the data folder. This way each test case will correspond to a different
# method and look like a separate test

testDataRoot = pathlib.Path(__file__).parent / "cases"
for folder in testDataRoot.iterdir():
    setattr(TextComparisonTests, "test_"+folder.name, lambda self: self._do_test_case(folder))



if __name__ == '__main__':
    logging.basicConfig()
    logger.setLevel(logging.DEBUG)
    unittest.main()
