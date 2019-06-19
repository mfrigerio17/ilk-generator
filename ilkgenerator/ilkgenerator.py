import logging, os, argparse, yaml

import robmodel.convert.importers.urdf   as urdfImporter

kindslSupport = True
try:
    import robmodel.convert.importers.kindsl as kindslImporter
except ImportError as e:
    kindslImportErrorMsg = e.msg
    kindslSupport = False

import robmodel

from gr import motions

from ilkgenerator import query, solvermodel, generator, robotconstants

log = logging.getLogger(__name__)

default_outdir = "/tmp/ilk"

def main():
    formatter = logging.Formatter('%(levelname)s : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    log.setLevel(logging.WARN)
    log.addHandler(handler)

    argparser = argparse.ArgumentParser(description='Generate ILK solver models')

    opts_outer_group = argparser.add_mutually_exclusive_group(required=True)

    opts_outer_group.add_argument('-y', '--yaml',   dest='yaml', action='store_true', help='Use a model split in several YAML files (dev option)')
    opts_outer_group.add_argument('-u', '--urdf',   metavar='PATH', dest='urdf',   help='Use a URDF robot model')
    opts_outer_group.add_argument('-k', '--kindsl', metavar='PATH', dest='kindsl', help='Use a KinDSL robot model')

    argparser.add_argument('-q', '--query', metavar='QUERY', dest='query',
            help='the YAML file containing a query (defaults to a random FK solver)')
    argparser.add_argument('-odir', '--output-dir', metavar='ODIR', dest='odir',
            default = default_outdir,
            help='the directory where to put the generated files (defaults to ' + default_outdir + ')')

    argparser_yaml = argparse.ArgumentParser(prog='<program> --yaml')
    argparser_yaml.add_argument('robot', metavar='robot-model',
             help='path of the robot model input file')
    argparser_yaml.add_argument('numbering', metavar='num-scheme',
             help='path of the numbering scheme input file')
    argparser_yaml.add_argument('geometry', metavar='geometry',
             help='path of the geometric numerical data input file')

    (args, extra) = argparser.parse_known_args()
    if args.yaml :
        yaml_args = argparser_yaml.parse_args(extra)

        istream = open(yaml_args.robot)
        connectivity = robmodel.robot.fromYAML( istream )
        istream.close()

        istream = open(yaml_args.numbering)
        numscheme = robmodel.ordering.fromYAML( istream )
        istream.close()

        robotmodel  = robmodel.ordering.Robot( connectivity, numscheme )
        robotframes = robmodel.frames.RobotDefaultFrames(robotmodel, [])

        istream = open(yaml_args.geometry)
        data    = yaml.load(istream)
        istream.close()
        posesSpec = motions.PosesSpec.fromDict(data)
        geometrymodel = robmodel.geometry.Geometry(robotmodel, robotframes, posesSpec)

    elif args.urdf :
        urdffile = open( args.urdf )
        urdf = urdfImporter.URDFWrapper(urdffile)
        connectivity, ordering, robotframes, geometrymodel = urdfImporter.convert(urdf)
        robotmodel = ordering # this is the model composed of connectivity plus numbering scheme

    elif args.kindsl :
        if not kindslSupport :
            raise RuntimeError("KinDSL support not available!, are you perhaps missing textX in your Python environment?"+
                               " The import error was: " + kindslImportErrorMsg)
        connectivity, ordering, robotframes, geometrymodel = kindslImporter.convert(args.kindsl)
        robotmodel = ordering # this is the model composed of connectivity plus numbering scheme

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
