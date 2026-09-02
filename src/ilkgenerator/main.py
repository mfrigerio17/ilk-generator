import logging, pathlib, argparse, yaml

import robmodel
import rmt.rmt as rmtool

from kgprim import motions

from ilkgenerator import query, solvermodel, generator, robotconstants

log = logging.getLogger(__name__)

default_outdir = "/tmp/ilk"

def generateILKFiles(outdirpath, geometrymodel, sweepingsolvers, iksolvers):
    ikSolverModels = []
    for sspecs in iksolvers :
        solver = solvermodel.IKSolverModel(sspecs)
        requiredFK = solver.requiredFK
        if requiredFK in sweepingsolvers :
            i = sweepingsolvers.index( requiredFK )
            fk = sweepingsolvers[i]
            # Force the IK to reference the FK solver we just found
            solver.requiredFK = fk
            log.info("The FK solver required by IK solver '{0}' is already available (solver '{1}')".format(solver.name, fk.name))
        else :
            sweepingsolvers.append( requiredFK )
            log.info("Generating the FK solver required by IK solver '{0}'".format(solver.name))

        ikSolverModels.append( solver )

    for sspecs in sweepingsolvers :
        solver = solvermodel.FKSolverModel(sspecs)
        gen = generator.SweepingSolverGenerator(solver)
        lua = gen.lua()
        with open(outdirpath / (solver.name + ".ilk"), mode='w') as ostream:
            ostream.write(lua)

    for solver in ikSolverModels :
        gen = generator.IKGenerator(solver)
        lua = gen.lua()
        with open(outdirpath / (solver.name + ".ilk"), mode='w') as ostream:
            ostream.write(lua)

    kk = robotconstants.asLuaTable(geometrymodel)
    with open(outdirpath / "model-constants.lua", mode='w') as ostream:
        ostream.write(kk)


def main():
    formatter = logging.Formatter('%(levelname)s : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    log.setLevel(logging.WARN)
    log.addHandler(handler)

    argparser = argparse.ArgumentParser(prog="ilkgen", description='Generate ILK solver models')

    rmtool.setRobotArgs(argparser)

    argparser.add_argument('-q', '--query', metavar='QUERY', dest='query',
            help='the YAML file containing a query (defaults to a random FK solver)')
    argparser.add_argument('-o', '--output-dir', metavar='ODIR', dest='odir',
            default = default_outdir,
            help=f"the directory where to put the generated files (defaults to '{default_outdir}')")

    args = argparser.parse_args()

    _, robotmodel, robotframes, geometrymodel, _, params = rmtool.getmodels(args.robot, args.params)[0:6]
    ## 'robotmodel' is the model composed of connectivity plus numbering scheme
    rmtool._resolve_parameters(geometrymodel.posesModel.poses, params)

    if args.query :
        with open(args.query) as istream:
            userq = query.queryFromYAML( istream )
    else :
        userq = query.defaultQuery(robotmodel)

    try:
        qparser = query.QueryParser(robotmodel, robotframes, None)
        sweepingsolvers, iksolvers = qparser.validate(userq)
    except Exception as e:
        log.error("Parsing exception: %s", e)
        return -1

    outdir = pathlib.Path(args.odir)
    if not outdir.exists():
        outdir.mkdir(parents=True)

    generateILKFiles(outdir, geometrymodel, sweepingsolvers, iksolvers)

