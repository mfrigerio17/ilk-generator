import logging, os, argparse, yaml

import robmodel
import rmt.rmt as rmtool

from kgprim import motions

from ilkgenerator import query, solvermodel, generator, robotconstants

log = logging.getLogger(__name__)

default_outdir = "/tmp/ilk"

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
            help='the directory where to put the generated files (defaults to ' + default_outdir + ')')
    argparser.add_argument('-e', '--experimental', dest='exp', action='store_true',
            help='?!?')

    args = argparser.parse_args()

    connectivity, tree, robotframes, geometrymodel, inertia, params = rmtool.getmodels(args.robot)
    robotmodel = tree # this is the model composed of connectivity plus numbering scheme

    if args.exp :
        from ilkgenerator.imp.robot import RobotModel
        from ilkgenerator.imp import luabridge
        rmodel = RobotModel(geometrymodel)
        luamodel = luabridge.lua_friendly_robot(rmodel)
        gen = luabridge.load_module('generator.lua')
        engine = luabridge.load_module('id.lua')
        tape = engine(luamodel)
        text = gen(luamodel, tape)
        ostream = open(args.odir + "/" + "try" + ".ilk", mode='w')
        ostream.write(text)
        ostream.close()
        return

    if args.query :
        istream = open(args.query)
        userq   = query.queryFromYAML( istream )
        istream.close()
    else :
        userq = query.defaultQuery(robotmodel)

    qparser = query.QueryParser(robotmodel, robotframes, None)
    sweepingsolvers, iksolvers = qparser.validate(userq)

    if not os.path.exists(args.odir) :
        os.makedirs(args.odir)

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
        ostream = open(args.odir + "/" + solver.name + ".ilk", mode='w')
        ostream.write(lua)
        ostream.close()

    for solver in ikSolverModels :
        gen = generator.IKGenerator(solver)
        lua = gen.lua()
        ostream = open(args.odir + "/" + solver.name + ".ilk", mode='w')
        ostream.write(lua)
        ostream.close()

    kk = robotconstants.asLuaTable(geometrymodel)
    ostream = open(args.odir + "/model-constants.lua", mode='w')
    ostream.write(kk)
    ostream.close()
