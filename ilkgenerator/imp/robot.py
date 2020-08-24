from enum import Enum, auto
import itertools

import robmodel.connectivity
import robmodel.treeutils
import robmodel.jposes

import kgprim.values as expr



class RobotModel:
    '''
    The robot model data structure used by `robcogen.*` classes and functions.

    This class composes together the different aspects of a robot model as
    represented by the classes of the `robmodel` package.

    It also adds some metadata and functions which are required by the robcogen
    code generators.
    '''

    def __init__(self, geometryModel, inertia = None, floatingBase=False):
        self.fb = floatingBase
        self.frames    = geometryModel.framesModel
        self.tree      = geometryModel.connectivityModel
        self.treeutils = robmodel.treeutils.TreeUtils(self.tree)
        self.kinematics= RobotKinematics(geometryModel)
        self.inertia   = None
        if inertia is not None :
            self.inertia = RobotInertia(self, inertia)
        #TODO save the link-frame inertia properties

        start = 1
        if floatingBase : start = 0
        self.movLinks = [ self.tree.codeToLink[l] for l in range(start, self.tree.nB)]

        self.hasPrismaticJoint = any([j.kind==robmodel.connectivity.JointKind.prismatic
                                            for j in self.tree.joints.values()])
        self.hasRevoluteJoint  = any([j.kind==robmodel.connectivity.JointKind.revolute
                                            for j in self.tree.joints.values()])


    @property
    def name(self):
        return self.tree.name

    def movingLinks(self):
        return self.movLinks

    def allConstantsIter(self):
        return itertools.chain(
                self.inertia.constants.keys(),
                self.kinematics.constants.keys() )


class IPField(Enum):
    '''
    Enumeration of all the scalar values of a rigid body inertia
    '''
    mass = 1
    comx = 2
    comy = 4
    comz = 8
    ixx  = 16
    iyy  = 32
    izz  = 64
    ixy  = 128
    ixz  = 256
    iyz  = 512

ipgetter = {
    IPField.mass : (lambda bodyInertia : bodyInertia.mass),
    IPField.comx : (lambda bodyInertia : bodyInertia.com.x),
    IPField.comy : (lambda bodyInertia : bodyInertia.com.y),
    IPField.comz : (lambda bodyInertia : bodyInertia.com.z),
    IPField.ixx  : (lambda bodyInertia : bodyInertia.moments.ixx),
    IPField.iyy  : (lambda bodyInertia : bodyInertia.moments.iyy),
    IPField.izz  : (lambda bodyInertia : bodyInertia.moments.izz),
    IPField.ixy  : (lambda bodyInertia : bodyInertia.moments.ixy),
    IPField.ixz  : (lambda bodyInertia : bodyInertia.moments.ixz),
    IPField.iyz  : (lambda bodyInertia : bodyInertia.moments.iyz)
}

class RobotInertia:
    def __init__(self, robot, inertia):
        self.data = inertia.inertia # the by-link-name map

        self.parameters = {}
        self.constants  = {}
        self.pflags = {}
        self.cflags = {}

        for linkName in self.data:
            self.cflags[linkName] = set()
            self.pflags[linkName] = RobotInertia.ParametricFlags()
            self.registerArguments(robot.tree.links[linkName], self.data[linkName])

    def isParameter(self, candidate):
        return candidate in self.parameters

    def registerArguments(self, link, ip):
        for __, field in IPField.__members__.items():
            property = ipgetter[field](ip)

            if isinstance(property, expr.Expression) :
                rtexpr = RobotInertia.RoundTrippableArgument(property, link, field)
                quantity = property.arg

                if isinstance(quantity, expr.Parameter) :
                    if quantity not in self.parameters :
                        self.parameters[quantity] = RobotQuantityMetadata(quantity)
                    self.parameters[quantity].addExpression( rtexpr )
                    self.pflags[link.name].add( field )

                elif isinstance(quantity, expr.Constant) :

                    if quantity not in self.constants :
                        self.constants[quantity] = RobotQuantityMetadata(quantity)
                    self.constants[quantity].addExpression( rtexpr )
                    self.cflags[link.name].add( field )

    class RoundTrippableArgument:
        def __init__(self, expression, link, field):
            self.expression = expression
            self.link  = link
            self.field = field

        def symbolic(self):
            return self.expression.expr

        def isRotation(self): return False

        def __eq__(self, rhs):
            return (isinstance(rhs, RoundTrippableArgument) and
                    (self.expression == rhs.expression) and
                    (self.link == rhs.link) and
                    (self.field == rhs.field))
        def __hash__(self) :
            return hash(self.expression) + 7*hash(self.link) + 11*hash(self.field)

    class ParametricFlags:
        com = IPField.comx.value + IPField.comy.value + IPField.comz.value
        im  = IPField.ixx.value + IPField.iyy.value + IPField.izz.value +\
              IPField.ixy.value + IPField.ixz.value + IPField.iyz.value

        def __init__(self):
            self.flags = 0

        def add(self, field):
            self.flags = self.flags + field.value

        def allParametric(self):
            return self.parametricMass() and self.parametricCoM() and self.parametricTensor()

        def parametricMass(self):
            return bool(self.flags & IPField.mass.value)

        def parametricCoM(self):
            return bool(self.flags & RobotInertia.ParametricFlags.com)

        def parametricTensor(self):
            return bool(self.flags & RobotInertia.ParametricFlags.im)


from kgprim.motions import MotionStep

class RobotKinematics:
    def __init__(self, in_geometry):
        jointPoses = robmodel.jposes.JointPoses(in_geometry.connectivityModel, in_geometry.framesModel)

        self.poseSpecByPose = {**in_geometry.byPose, **jointPoses.poseSpecByPose}
        self.jointPosesByJoint = jointPoses.poseSpecByJoint
        self.geomPosesByJoint  = in_geometry.byJoint
        self.jointToSymVar = jointPoses.jointToSymVar
        self.symVarToJoint = jointPoses.symVarToJoint
        self.baseFrame = in_geometry.framesModel.linkFrames[ in_geometry.connectivityModel.base ]

        self.constants = {}
        self.parameters= {}
        self.registerArguments()

    def allPosesSpecs(self):
        return self.poseSpecByPose.values()

    def getPoseSpec(self, pose):
        return self.poseSpecByPose[pose]

    def getJointWrtPredecessorPose(self, joint):
        return self.geomPosesByJoint[joint]

    def getSuccessorWrtJointPose(self, joint):
        return self.jointPosesByJoint[joint]

    def registerArguments(self):
        for poseSpec in self.poseSpecByPose.values() :
            motionPath = poseSpec.motion
            for motionSequence in motionPath.sequences :
                for motionStep in motionSequence.steps :
                    amount = motionStep.amount
                    if isinstance(amount, expr.Expression) :
                        meta = RobotKinematics.RoundTrippableArgument(amount, motionStep, poseSpec.pose, )
                        quantity = amount.arg

                        if isinstance(quantity, expr.Parameter) :
                            if quantity not in self.parameters :
                                self.parameters[quantity] = RobotQuantityMetadata(quantity)
                            self.parameters[quantity].addExpression( meta )

                        elif isinstance(quantity, expr.Constant) :
                            if quantity not in self.constants :
                                self.constants[quantity] = RobotQuantityMetadata(quantity)
                            self.constants[quantity].addExpression( meta )

    class RoundTrippableArgument:
        def __init__(self, expression, motionStep, pose):
            self.expression  = expression
            self.amountOf    = motionStep
            self.takesPartIn = pose
            # assert( self.amountOf.amount == self.expression )

        def symbolic(self):
            return self.expression.expr

        def isRotation(self):
            return self.amountOf.kind == MotionStep.Kind.Rotation

        def __eq__(self, rhs):
            if not isinstance(rhs, RoundTrippableArgument) :
                return False
            return ((self.expression == rhs.expression) and
                    (self.amountOf == rhs.amountOf) and
                    (self.takesPartIn == rhs.takesPartIn))

        def __hash__(self) :
            return hash(self.expression) + 7*hash(self.amountOf) + 11*hash(self.takesPartIn)


class RobotQuantityMetadata:
    def __init__(self, quantity):
        '''
        Parameters:

        - `quantity`: either a `vpc.vpc.Parameter` or `vpc.vpc.Constant` which
           is part of the definition a property of the robot model
        '''
        self.quantity = quantity
        self.expressions = set()

    def addExpression(self, expr):
        self.expressions.add(expr)

    def __eq__(self, rhs):
        if not isinstance(rhs, RobotQuantityMetadata) :
            return False
        return self.quantity == rhs.quantity

    def __hash__(self) :
        return hash(self.quantity)

