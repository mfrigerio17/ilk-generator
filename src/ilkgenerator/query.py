'''
Created on Nov 16, 2018

@author: marco
'''
from enum import Enum
from collections import namedtuple
import yaml
import logging

from ilkgenerator import solvermodel
from kgprim import core as gr

log = logging.getLogger(__name__)



def queryFromDictionary(data):
    robotname = data['robot']
    sweepingSolvers = []
    ikSolvers = []
    for solver in data['solvers']:
        solverKind = solver['kind']
        if solverKind == 'sweeping' :
            sweepingSolvers.append( _FKSolver.fromDict(solver) )
        elif solverKind == 'IK':
            ikSolvers.append( _IKSolver.fromDict(solver) )
        else:
            log.error("Unknown solver kind '{0}'".format(solverKind))

    return _Query(robotname, sweepingSolvers, ikSolvers)


def queryFromYAML(istream):
    data = yaml.safe_load(istream)
    return queryFromDictionary(data)



# These named tuples are meant to mirror the expected format of the
# dictionary with input data
VelSpecs = namedtuple('_Velocity', ['target', 'reference', 'kind', 'cframe'])
JacSpecs = namedtuple('_Jacobian', ['target', 'reference'])

class IKLevel(Enum):
    position = 0
    velocity = 1

class CartesianConfigurationSpace(Enum):
    linear = 0,
    angular = 1,
    pose = 2




class _FKSolver:
    '''A placeholder for a sweeping solver declarative model, as found in a query.

    For internal use.
    '''
    def __init__(self, name, kind, outputs):
        self.name = name
        self.kind = kind
        self.outputs = outputs

    def __str__(self):
        text = "solver {0:s} ('{1:s}' kind):\n".format(self.name, self.kind)
        text = text + "Poses: " + ", ".join([el.__str__() for el in self.outputs['poses']])
        text = text + "\n"
        text = text + "Veloc: " + ", ".join([el.__str__() for el in self.outputs['velocities']])
        return text


    @staticmethod
    def fromDict(data):
        outputs = {}
        queryout = data['outputs']
        if 'poses' in queryout :
            poses = []
            for pose in queryout['poses']:
                tgt = gr.Frame(pose['target'])
                ref = gr.Frame(pose['reference'])
                poses.append( gr.Pose(target=tgt, reference=ref) )
            outputs['poses'] = poses
        else :
            outputs['poses'] = []

        if 'velocities' in queryout :
            outputs['velocities'] = [VelSpecs(**vel) for vel in queryout['velocities']]
            # we expect the dictionary data to match the fields of the
            # named tuples thus we can use the **notation
        else :
            outputs['velocities'] = []

        if 'jacs' in queryout :
            outputs['jacobians'] = [JacSpecs(**jac) for jac in queryout['jacs']]
        else :
            outputs['jacobians'] = []

        return  _FKSolver(data['name'], data['kind'], outputs)




class _IKSolver():
    '''A placeholder for an inverse kinematics solver declarative model, as found in a query.

    '''
    def __init__(self, name, level, cfgSpace, tgtF, refF):
        self.name  =  name
        self.level = level
        self.cfgSpace = cfgSpace
        self.targetFrame = tgtF
        self.referenceFrame = refF

    def __str__(self):
        return "IK solver '{0}' at the {1} level, for {2} vectors, about frame {3} relative to {4}".format(
            self.name, self.level.name, self.cfgSpace.name,
            self.targetFrame.name, self.referenceFrame.name)

    @staticmethod
    def fromDict(data):
        name     = data['name']
        level    = IKLevel[ data['level'] ]
        cfgspace = CartesianConfigurationSpace[ data['cfgSpace'] ]
        frames   = data['frames']

        tgt = gr.Frame(frames['target'])
        ref = gr.Frame(frames['reference'])
        return _IKSolver(name, level, cfgspace, tgt, ref)




class _Query:
    '''A placeholder for a user query, basically a list of solver requests.

    For internal use
    '''
    def __init__(self, robotname, sweepingsolvers, iksolvers):
        self.robotname = robotname
        self.sweepingSolvers = sweepingsolvers
        self.ikSolvers = iksolvers

    def __str__(self):
        text = "_Query on robot {0:s}, about:\n".format(self.robotname)
        for s in self.solvers:
            text = text + s.__str__()+"\n"
        return text




def checkVelocitySpecs(robotFrames, robotPoints, v):
    '''Verifies the consistency of the given velocity specs against the given
    robot model.

    Returns an instance of gr.core.Velocity
    '''

    robot = robotFrames.robot
    cframe = robotFrames.framesByName.get(v.cframe, None)
    if v.kind == 'linear' :
        if v.target not in robotPoints:
            raise ValueError("The target of a linear velocity must be a point attached to the robot")
        tgt = robotPoints.attachedPoint(v.target)
        if v.reference in robotPoints :
            ref = robotPoints.attachedPoint(v.reference)
        else:
            if v.reference not in robot.links :
                raise ValueError("Unknown velocity reference '{0}'".format(v.reference))
            ref = robot.links[v.reference]

    elif v.kind == "6D" :
        if v.target in robot.links :
            tgt = robotFrames.linkFrames[ robot.links[v.target] ]
        elif v.target in robot.joints :
            tgt = robotFrames.jointFrames[ robot.joints[v.target] ]
        elif v.target in robotFrames.framesByName :
            tgt = robotFrames.framesByName[v.target]
        else:
            raise ValueError("The target for a linear+angular velocity must be a frame of the robot (offending key: '{0}')".format(v.target))

        if v.reference in robot.links :
            ref = robotFrames.linkFrames[ robot.links[v.reference] ]
        elif v.reference in robot.joints :
            ref = robotFrames.jointFrames[ robot.joints[v.reference] ]
        elif v.reference in robotFrames.framesByName :
            ref = robotFrames.framesByName[v.reference]
        else:
            raise ValueError("The reference for a linear+angular velocity must be a frame of the robot (offending key: '{0}')".format(v.reference))

        #TODO deal with the cases of the user specifying explicitly a coordinate frame
        if cframe == None:
            cframe = ref
            # this is the default for Euclidean velocities: the coordinates are
            # in the same frame as the reference of the velocity


    #elif v.kind == "spatial":
        # TODO
        # the target must be a link
        # if the reference is another frame on a link, the target is such a link,
        # and automatically the coordinate-frame is the reference as indicated by the user
        # If the user _also_ indicated a coordinate-frame, that is a mistake
    else:
        raise RuntimeError("Unknown velocity kind '{}'".format(v.kind))


    vel = gr.Velocity(tgt, ref)
    vel.kind = v.kind
    vel.cframe = cframe

    return vel



class QueryParser():
    '''
    A validator of the user queries, which produces a list of solver models.
    See module solvermodel.

    An instance must be constructed with robot model components covering
    connectivity and attached frames/points.
    '''

    def __init__(self, robotConnect, robotFrames, robotPoints):
        self.robot = robotConnect
        self.frames= robotFrames
        self.points= robotPoints
        self.robotModelsDict = {
            'robot' : self.robot,
            'frames': self.frames,
            'points': self.points
        }

    def validate(self, query):
        '''
        Check the given query against the robot model used to construct this instance.
        Returns a list of solvermodel.SolverSpecs instances.
        '''
        if query.robotname != self.robot.name :
            raise ValueError("Mismatching robot names: '{0}' (query) and '{1}' (robot model)".format(query.robotname, self.robot.name))

        sweepingsolvers = []
        for s in query.sweepingSolvers :
            poses = self.validatePoses(s.outputs['poses'])
            vels  = self.validateVelocities(s.outputs['velocities'])
            jacs  = self.validateJacobians (s.outputs['jacobians'])
            sweepingsolvers.append(
                solvermodel.FKSolverSpecs(
                    name= s.name, kind= s.kind,
                    rmodels = self.robotModelsDict,
                    requests= {'pose':poses,'velocity':vels, 'jacobian':jacs} ))
        iksolvers = []
        for s in query.ikSolvers :
            iksolvers.append( self.validateIKDeclarativeModel(s) )

        return sweepingsolvers, iksolvers

    def validatePoses(self, posesRequest) :
        poses = []
        for pose in posesRequest :
            target    = self.frames.getAttachedFrame(pose.target)
            reference = self.frames.getAttachedFrame(pose.reference)
            if target==None or reference==None :
                raise RuntimeError("Could not find attached frame " + pose.target.name + " or " + pose.reference.name)
            poses.append( gr.Pose(target, reference) )

        return poses

    def validateVelocities(self, velocitiesRequest):
        return [self._checkVelocity(v) for v in velocitiesRequest]


    def _checkVelocity(self, v):
        return checkVelocitySpecs(self.frames, self.points, v)

    def validateJacobians(self, jacs):
        # Any Jacobian pertains to a relative velocity. Let's construct it and
        # and check it
        ret = []
        for J in jacs:
            vel = VelSpecs(target=J.target, reference=J.reference, kind="6D", cframe="NA")
            vel = self._checkVelocity(vel)
            ret.append( solvermodel.JacobianSpecs(velocity=vel) )
        return ret

    def validateIKDeclarativeModel(self, ik):
        target = self.frames.getAttachedFrame(ik.targetFrame)
        if target == None :
            raise RuntimeError("Frame '{0}' does not seem to be attached to robot '{1}'"
                               .format(ik.targetFrame, self.frames.robot.name) )

        reference = self.frames.getAttachedFrame(ik.referenceFrame)
        if reference == None :
            raise RuntimeError("Frame '{0}' does not seem to be attached to robot '{1}'"
                               .format(ik.referenceFrame, self.frames.robot.name) )

        return solvermodel.IKSolverSpecs(
                 rmodels = self.robotModelsDict, name=ik.name, level=ik.level,
                 cfgSpace=ik.cfgSpace, targetFrame=target, referenceFrame=reference)


def defaultQuery(robot) :
    d = {}
    d['robot'] = robot.name
    solver = {}
    solver['name'] = 'default-fk'
    solver['kind'] = 'sweeping'
    outputs = {}
    pose = {}
    pose['reference'] = robot.base.name
    lastLink = list(robot.links.values())[-1]
    pose['target'] = lastLink.name

    outputs['poses'] = [ pose ]
    solver['outputs'] = outputs
    d['solvers'] = [ solver ]
    return queryFromDictionary(d)



