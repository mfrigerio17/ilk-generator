'''
Created on Nov 16, 2018

@author: marco
'''

from collections import namedtuple

from ilkgenerator import optcompose
from ilkgenerator.optcompose import HomogenoeusComposable
from ilkgenerator import query
from ilkgenerator import jacobians
from ilkgenerator import utils

from kgprim import core as gr
from robmodel.frames import FrameRelationKind
from robmodel.connectivity import JointKind


#TODO treat poses and velocities consistently: separate the semantic primitives
# from enriched specifications required in a solver model. Do not mix the two
# levels.



class JacobianSpecs:
    def __init__(self, velocity):
        self.velocity = velocity

    def __eq__(self, rhs):
        return (isinstance(rhs, JacobianSpecs) and
                self.velocity == rhs.velocity )
    def __hash__(self):
        return 113 * hash(self.velocity)
    def __str__(self):
        return "Jacobian for velocity {0}".format(self.velocity)
    def __repr__(self):
        return self.__str__()



VelocitySpecs = namedtuple('VelocitySpecs', ['vel', 'kind', 'cframe'])

JointVel = namedtuple('JointVel', ['joint', 'vel', 'polarity'])

class FKSolverSpecs:
    '''Data required to specify a declarative model of a FK solver.

    The identifying attributes include a name, the robot model(s) the solver
    refers to, and the quantities to be computed. These include relative poses,
    relative velocities, geometric Jacobians.
    '''

    def __init__(self,**kwargs):
        self.name = kwargs['name']
        self.kind = kwargs['kind']
        self.rmodels  = kwargs['rmodels']
        self.requests = kwargs['requests']

        if 'pose' not in self.requests :
            self.requests['pose'] = []
        if 'jacobian' not in self.requests :
            self.requests['jacobian'] = []
        if 'velocity' not in self.requests :
            self.requests['velocity'] = []

        self.poses = tuple( self.requests['pose'] )
        self.jacs  = tuple( self.requests['jacobian'] )
        self.vels  = tuple( self.requests['velocity'] )

    def __eq__(self, rhs):
        almost = (isinstance(rhs, FKSolverSpecs) and
               #self.name == rhs.name and
               self.kind == rhs.kind and
               self.rmodels['robot'].name == rhs.rmodels['robot'].name) # weak check...
        ret = False
        if almost :
            ret = (self.poses==rhs.poses) and (self.jacs==rhs.jacs) and (self.vels==rhs.vels)
        return ret

    def __hash__(self) :
        return(#47 * hash(self.name) +
               31 * hash(self.kind) +
               79 * hash(self.rmodels['robot'].name) +
               11 * hash(self.poses) +
               13 * hash(self.jacs) +
               83 * hash(self.vels))

    def __str__(self):
        return "Solver '{0}' of {1} kind, for robot {2}, requesting: {3} {4}".format(
            self.name, self.kind, self.rmodels['robot'].name,
            self.poses, self.jacs)


class _ComposablePose(HomogenoeusComposable):
    def __init__(self, pose):
        super().__init__([pose])
        self.originalPose = pose

    @property
    def target(self)   : return self.originalPose.target
    @property
    def reference(self): return self.originalPose.reference

    def compose(self, others):
        current = self.originalPose
        for ot in others :
            current = gr.poseCompose(current, ot.originalPose)
        return _ComposablePose(current)

class _ComposableVelocity(HomogenoeusComposable):
    def __init__(self, velocity):
        super().__init__([velocity])
        self.v = velocity

    @property
    def target(self)   : return self.v.target
    @property
    def reference(self): return self.v.reference

    def compose(self, others):
        current = self.v
        for ot in others :
            current = gr.velCompose(current, ot.v)
        return _ComposableVelocity(current)


class FKSolverModel:
    '''A declarative model of a FK solver, with information about the optimal
    pose/velocity compositions to perform.

    An instance must be constructed from a FKSolverSpecs instance.
    '''

    def __init__(self, solverSpec):
        self.name    = solverSpec.name
        self.rmodels = solverSpec.rmodels
        self.constPoses = set()
        self.jointPoses = set()
        self.output = solverSpec.requests

        framesModel = self.rmodels['frames']

        allPoses = set( self.output['pose'] ) # shallow copy of the list

        # Geometric Jacobians require specific relative poses to be computed, so
        # we must add those to the poses explicitly requested in the query.
        self.geometricJacobians = []
        for J in self.output['jacobian'] :
            jac = jacobians.GeometricJacobian(framesModel, J.velocity)
            allPoses.update(jac.jointPoses)
            allPoses.add(jac.targetPose)
            self.geometricJacobians.append(jac)
        self.output['jacobian'] = self.geometricJacobians

        self.jointVelocities = {}
        velComposePaths = [self.velocityPath(v) for v in self.output['velocity']]
        self.velComposes = optcompose.allComposes( velComposePaths )
        self.velBinaryComposes = []

        # The composition of velocities, on the other hand, requires certain
        # coordinate transforms. We can again add some relative poses to achieve
        # the desired effect, relying on the assumption that relative poses
        # will be concretely represented by usable coordinate transforms, e.g.
        # homogeneous transforms. This is a bit of hack, as the two concepts
        # (relative pose and coordinate transform) should be probably kept
        # separate.
        for vcomp in self.velComposes :
            for vcomp in vcomp.asSequenceOfBinaryCompositions() :
                #print(vcomp.arg1.target.name + " " + vcomp.arg1.reference.name)
                #print(vcomp.arg2.target.name + " " + vcomp.arg2.reference.name)
                #print("require transform from {0} to {1}".format(vcomp.arg2.target.name,vcomp.arg1.target.name))
                # Now the hack gets uglier, because we are also assuming that
                # this specific pose (see code) is the one encoding the appropriate
                # coordinate transform...
                tgtF = framesModel.framesByName[vcomp.arg2.target.name]
                refF = framesModel.framesByName[vcomp.arg1.target.name]
                pose = gr.Pose(target=tgtF, reference=refF)
                allPoses.add( pose )
                vcomp.pose = pose
                self.velBinaryComposes.append(vcomp)

        # The second argument of any binary velocity composition, is the one that
        # gets coordinate-transformed, thus it must have been computed explicitly
        # before the composition. If it is a velocity across a joint, then we
        # have to mark it for explicit computation, because joint velocities are
        # in general not computed explicitly for optimization purposes.
        self.jointVelocitiesExplicit = set()
        for vbc in self.velBinaryComposes :
            if vbc.arg2.v in self.jointVelocities.keys() :
                self.jointVelocitiesExplicit.add( vbc.arg2.v )
        # Consider the corner case in which the desired output velocity is a
        # joint velocity
        for v in self.output['velocity'] :
            if v in self.jointVelocities.keys() :
                self.jointVelocitiesExplicit.add( v )

        # Joint velocities with opposite polarity (that is, velocity of predecessor
        # relative to successor), require the coordinate transform from joint
        # frame to predecessor frame. We add here the corresponding pose to make
        # sure the solver will include it
        for v in self.jointVelocities.values() :
            if v.polarity == -1 :
                refF = self.robotFrames.framesByName[v.vel.target.name]
                tgtF = self.robotFrames.framesByName[v.joint.name]
                pose = gr.Pose(target=tgtF, reference=refF)
                allPoses.add( pose )

        poseComposePaths = [self._posePath(pose) for pose in allPoses ]
        self.poseComposes = optcompose.allComposes( poseComposePaths )

    @property
    def robot(self):
        return self.rmodels['robot']
    @property
    def robotFrames(self):
        return self.rmodels['frames']



    def _posePath(self, givenpose):
        '''
        The sequence of distance-1 poses equivalent to the given pose.

        Distance-1 poses involve adjacent frames, i.e. their value is a
        "primitive" value of the robot model.
        The returned sequence is constructed from the shortest path connecting
        the target frame and the reference frame of the original given pose.

        The returned sequence is in fact an optcompose.Path object.

        While building the sequence, this method also populates the sets of
        constant-poses and joint-poses of this instance.
        '''
        composablesList = []
        framesGraph = self.rmodels['frames']
        graphPath   = framesGraph.path(givenpose.target, givenpose.reference)
        tgt = givenpose.target
        for ref in graphPath[1:] :
            pose = _ComposablePose( gr.Pose(tgt, ref) )
            if framesGraph.kind(tgt, ref) is FrameRelationKind.acrossJoint :
                joint = framesGraph.joint(tgt, ref)
                if joint.kind == JointKind.fixed:
                    self.constPoses.add( pose )
                elif utils.isSupportedTypeAndNonFixed(joint):
                    pose.joint = joint
                    self.jointPoses.add( pose )
                else:
                    raise RuntimeError("Unsupported joint kind '{}', for joint '{}'"
                        .format(joint.kind, joint.name))
            else :
                self.constPoses.add( pose )
            #if not framesGraph.relativePoseIsIdentity(tgt, ref) :

            composablesList.append( pose )
            tgt = ref

        return optcompose.Path(composablesList)

    def velocityPath(self, v):
        ref = v.reference # should always be a robot link
        if v.kind == "6D" :
            tgt = v.target
        elif v.kind == "linear" :
            tgt = v.target.body
        else :
            raise RuntimeError("Unknown velocity kind")

        bodies = self.robot.links.values()
        if ref not in bodies or tgt not in bodies :
            raise RuntimeError("Fatal, a relative velocity must involve links "
                "of robot {0} (found '{1}' and '{2}')".format(
                    self.robot.name, tgt.name, ref.name))

        path = self.robot.path(ref, tgt)
        composablesList = []
        for tgt in path[1:] :
            vel = gr.Velocity(tgt, ref) # tgt and ref are neighbour links, so this should be a joint velocity
            joint = self.robot.linkPairToJoint(tgt, ref)
            polarity = 1
            if vel.target == self.robot.predecessor(joint) :
                polarity = -1
            jvel = JointVel(joint=joint, vel=vel, polarity=polarity)
            self.jointVelocities[vel] = jvel

            vel = _ComposableVelocity( vel )
            composablesList.append( vel )
            ref = tgt

        return optcompose.Path(composablesList, True)


# Data required to represent a declarative model of an IK solver
IKSolverSpecs = namedtuple('IKSolverSpecs', ['rmodels','name','level','cfgSpace','targetFrame','referenceFrame'])

class IKSolverModel():
    '''A declarative model of an IK solver.

    With respect to IKSolverSpecs, this class works out the specs of the FK
    solver that this IK solver would need for the numerical computations. '''

    def __init__(self, specs) :
        self._robot = specs.rmodels['robot']
        self._frames= specs.rmodels['frames']
        self.name     = specs.name
        self.level    = specs.level
        self.cfgSpace = specs.cfgSpace
        self.targetFrame = specs.targetFrame
        self.referenceFrame = specs.referenceFrame

        fkSolverName = "fk__" + self.name
        pose = gr.Pose(target=specs.targetFrame, reference=specs.referenceFrame)
        fkouts = {}
        fkouts['pose'] = [ pose ]

        tgtL = specs.targetFrame.body
        refL = specs.referenceFrame.body
        velSpecs = query.VelSpecs(target=tgtL.name, reference=refL.name, kind="6D", cframe="NA")
        vel = query.checkVelocitySpecs(specs.rmodels['frames'], None, velSpecs)
        fkouts['jacobian'] = [JacobianSpecs(velocity=vel)]
        self.requiredFK = FKSolverSpecs(name=fkSolverName, kind="sweeping", rmodels=specs.rmodels, requests=fkouts)

    @property
    def robot(self):
        return self._robot
    @property
    def robotFrames(self):
        return self._frames
