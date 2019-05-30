'''
Created on Nov 22, 2018

@author: marco
'''

from mako.template import Template
from collections import namedtuple

from ilkgenerator import query

from gr import core as gr
from robmodel import frames

def poseIdentifier(pose):
    return pose.target.name + "__" + pose.reference.name

def velocityIdentifier(velocity):
    return "v__"+velocity.target.name+"__"+velocity.reference.name

def gJacobianIdentifier(gjac):
    return "J_" + gjac.velocity.target.name + "_" + gjac.velocity.reference.name

def commaSeparatedLines(sequence, lineTemplate, itemName, context):
    lineTPL = Template(lineTemplate)
    count = 0
    suffix = ","

    for item in sequence :
        context[itemName] = item
        context['loopi']  = count
        if count == len(sequence)-1 : suffix = ""
        yield lineTPL.render(**context) + suffix
        count = count + 1

def multiLineBlock(iterable, indent=""):
    tpl = Template('''
% for item in iterable :
${space}${item}
% endfor''')
    return tpl.render( iterable=iterable, space=indent )

def jointTypeStr(joint) :
    return joint.kind # by chance, the same string we use in the ILK

def directionTag(jointPose) :
    targetKind = jointPose.target.attrs['role']
    if targetKind is frames.FrameRole.linkRef :
        return "a_x_b"
    elif targetKind is frames.FrameRole.joint :
        return "b_x_a"
    else :
        raise RuntimeError("Got a joint-pose whose target is neither a link nor a joint frame")



class ILKGenerator():
    BlockSpec = namedtuple('BlockSpec', ['lineTemplate', 'singleItemName', 'context'])

    poseComposeBlock = BlockSpec(
        lineTemplate = '''{ op='pose-compose', arg1='${toID(c.arg1)}', arg2='${toID(c.arg2)}', res='${toID(c.result)}' }''',
        singleItemName = "c",
        context = {'toID' : poseIdentifier}
    )
    velComposeBlock = BlockSpec(
        lineTemplate = '''{ op='vel-compose', arg1='${toID(c.arg1)}', arg2='${toID(c.arg2)}', pose='${poseID(c.pose)}', res='${toID(c.result)}' }''',
        singleItemName = "c",
        context = {'toID' : velocityIdentifier, 'poseID' : poseIdentifier}
    )
    explicitJointVelocitiesBlock = BlockSpec(
        lineTemplate = '''{ op='vel-joint', arg='${toID(jv)}' }''',
        singleItemName = "jv",
        context = {'toID' : velocityIdentifier}
    )

    constPosesBlock = BlockSpec(
        lineTemplate = '''${toID(pose)}={}''',
        singleItemName = "pose",
        context = {'toID' : poseIdentifier}
    )
    jointPosesBlock = BlockSpec(
        lineTemplate = '''${toID(pose)} = { jtype='${jtype(pose.joint)}', dir='${dirTag(pose)}', input=${jnum(pose.joint)} }''',
        singleItemName = "pose",
        context = {'toID'  : poseIdentifier,
                   'dirTag': directionTag,
                   'jtype' : jointTypeStr}
    )

    outputPosesBlock = BlockSpec(
        lineTemplate = '''${toID(pose)} = {otype='pose', usersort=${next(oindex)} }''',
        singleItemName = "pose",
        context = {'toID' : poseIdentifier}
    )
    outputVelocitiesBlock = BlockSpec(
        lineTemplate = '''${toID(vel)} = {otype='velocity', usersort=${next(oindex)} }''',
        singleItemName = "vel",
        context = {'toID' : velocityIdentifier}
    )
    outputJacobiansBlock = BlockSpec(
        lineTemplate = "${toID(gjac)} = {otype='jacobian', usersort=${next(oindex)} }",
        singleItemName = "gjac",
        context = {'toID' : gJacobianIdentifier}
    )


    def __init__(self, solvermodel):
        poseComposes = []
        for composition in solvermodel.poseComposes :
            poseComposes.extend( composition.asSequenceOfBinaryCompositions() )

        self.explicitJointVelocities = solvermodel.jointVelocitiesExplicit
        self.velComposes = solvermodel.velBinaryComposes
        self.poseComposes = poseComposes
        self.solverModel  = solvermodel

        # The joint coordinate for 0-based sequences should be the joint code
        # in the robot model -1. This is because the regular numbering for joints
        # start with 1, which is the joint connecting the base (#0) with the
        # link #1.
        ILKGenerator.jointPosesBlock.context['jnum'] = lambda joint : solvermodel.robot.jointNum(joint)-1

        def outputIndex(self):
            total = sum( [len(block) for block in self.solverModel.output.values() ] )
            for i in range(0,total) :
                yield i+1
        outputVarIndexGenerator = outputIndex(self)
        ILKGenerator.outputPosesBlock.context['oindex'] = outputVarIndexGenerator
        ILKGenerator.outputVelocitiesBlock.context['oindex'] = outputVarIndexGenerator
        ILKGenerator.outputJacobiansBlock.context['oindex'] = outputVarIndexGenerator

        self.jointVelocitiesBlock = ILKGenerator.BlockSpec(
            lineTemplate = "${line(jvel)}",
            singleItemName = "jvel",
            context = {'line' : self.jointVelocityLine}
        )
        self.jacobiansBlock = ILKGenerator.BlockSpec(
            lineTemplate = "${line(J)}",
            singleItemName = "J",
            context = {'line' : self.jacobianBlock}
        )

    def commaSepLines(self, sequence, spec):
        return commaSeparatedLines(sequence, spec.lineTemplate, spec.singleItemName, spec.context)


    def jacobianBlock(self, J):
        Jid = gJacobianIdentifier(J)

        column_op = ILKGenerator.BlockSpec(
            lineTemplate = '''{ op='GJac-col', jtype='${jtype(joints[j])}', jac='${gjac_id}', col=${jnum(joints[j])}, joint_pose='${poseID(jposes[j])}', polarity=${polarities[j]} }''',
            singleItemName = "j",
            context = { "jtype"   : jointTypeStr,
                        "gjac_id" : Jid,
                        "jnum"    : self.solverModel.rmodels['robot'].jointNum,
                        "poseID"  : poseIdentifier,
                        "joints"  : J.joints,
                        "jposes"  : J.jointPoses,
                        "polarities" : J.polarities
                    }
        )

        templateText = '''
        { op='geom-jacobian', name='${gjac_id}', pose='${ee_pose}' },
    % for col_op in columnOps :
        ${col_op}
    % endfor'''
        context = {
            "gjac_id"   : Jid,
            "ee_pose"   : poseIdentifier(J.targetPose),
            "columnOps" : self.commaSepLines(range(0,len(J.joints)), column_op )
        }
        return Template(templateText).render(**context)


    def jointVelocityLine(self, jvel):
        tpl = ""
        poseid = ""
        if jvel.polarity == -1 :
            refF = self.solverModel.robotFrames.framesByName[jvel.vel.target.name]
            tgtF = self.solverModel.robotFrames.framesByName[jvel.joint.name]
            pose = gr.Pose(target=tgtF, reference=refF)
            poseid = poseIdentifier(pose)
            tpl = "${velid} = { jtype='${jtype}', index=${jnum}, polarity=-1, ctransform='${pose}' }"
        else :
            tpl = "${velid} = { jtype='${jtype}', index=${jnum}, polarity=1 }"

        context = {'velid' : velocityIdentifier(jvel.vel),
                   'jtype' : jointTypeStr(jvel.joint),
                   'jnum'  : self.solverModel.robot.jointNum(jvel.joint),
                   'pose'  : poseid }
        return Template(tpl).render(**context)




    def lua(self):

        ops_blocks = [ self.poseComposes, self.explicitJointVelocities, self.velComposes, self.solverModel.geometricJacobians]
        out_blocks = [ self.solverModel.output['pose'],
                       self.solverModel.output['velocity'],
                       self.solverModel.output['jacobian'] ]

        def separator(blocks):
            b = 0
            somethingBefore = len( blocks[b] ) > 0
            for b in range(1, len(blocks) ) :
                sep = ""
                currentNonEmpty = len( blocks[b] ) > 0
                if somethingBefore :
                    if currentNonEmpty :
                        sep = ","
                else :
                    somethingBefore = currentNonEmpty
                yield sep

        realJointsCount = len(self.solverModel.robot.joints)
        for j in self.solverModel.robot.joints.values() :
            if j.kind == "fixed" :
                realJointsCount -= 1
        template = '''
return {
    solverid = '${solver.name}',
    solver_type = 'forward',
    robot_name = '${solver.robot.name}',
    joint_space_size = ${realJointsCount},
    poses = {
        constant = {
        % for pose in const_poses :
            ${pose}
        % endfor
        },
        joint = {
        % for pose in joint_poses :
            ${pose}
        % endfor
        }
    },
    joint_velocities = {
        % for jvel in joint_velocities :
        ${jvel}
        % endfor
    },
    ops = {
    % for composition in pose_composes :
        ${composition}
    % endfor
    ${ next(ops_separator) }

    % for jointVel in explicit_joint_velocities :
        ${jointVel}
    % endfor
    ${ next(ops_separator) }

    % for velcompose in vel_composes :
        ${velcompose}
    % endfor
    ${ next(ops_separator) }

    % for gjac in jacobians :
${gjac}
    % endfor
    },

    outputs = {
    % for p in out_poses :
        ${p}
    % endfor
    ${ next(out_separator) }

    % for v in out_velocities :
        ${v}
    % endfor
    ${ next(out_separator) }

    % for J in out_jacobians :
        ${J}
    % endfor
    }
}
'''
        t = Template(template)
        context = {
            'this' : self,
            'solver': self.solverModel,
            'realJointsCount' : realJointsCount,

            'const_poses'  : self.commaSepLines(self.solverModel.constPoses, ILKGenerator.constPosesBlock),
            'joint_poses'  : self.commaSepLines(self.solverModel.jointPoses, ILKGenerator.jointPosesBlock),
            'joint_velocities' : self.commaSepLines(self.solverModel.jointVelocities.values(), self.jointVelocitiesBlock),

            'pose_composes': self.commaSepLines(self.poseComposes, ILKGenerator.poseComposeBlock),
            'explicit_joint_velocities' : self.commaSepLines(self.explicitJointVelocities, ILKGenerator.explicitJointVelocitiesBlock),
            'vel_composes' : self.commaSepLines(self.velComposes, ILKGenerator.velComposeBlock),
            'jacobians' : self.commaSepLines(self.solverModel.geometricJacobians, self.jacobiansBlock),

            'out_poses'    : self.commaSepLines(self.solverModel.output['pose'], ILKGenerator.outputPosesBlock),
            'out_velocities': self.commaSepLines(self.solverModel.output['velocity'], ILKGenerator.outputVelocitiesBlock),
            'out_jacobians': self.commaSepLines(self.solverModel.output['jacobian'], ILKGenerator.outputJacobiansBlock),
            'ops_separator' : separator( ops_blocks ),
            'out_separator' : separator( out_blocks )
        }
        return( t.render(**context) )


class IKGenerator():
    def __init__(self, ikSolverModel):
        self.declarativeModel = ikSolverModel


    def lua(self):
        levels = {
            query.IKLevel.position : "pos",
            query.IKLevel.velocity : "vel"
        }
        spaces = {
            query.CartesianConfigurationSpace.linear  : "linear",
            query.CartesianConfigurationSpace.angular : "angular",
            query.CartesianConfigurationSpace.pose    : "pose",
        }
        templateText = '''
return {
        solverid = '${dm.name}',
        solver_type = 'inverse',
        robot_name = '${dm.robot.name}',
        kind='${level}',
        vectors='${space}',
        target='${dm.targetFrame.name}',
        reference='${dm.referenceFrame.name}',
        fk='${dm.requiredFK.name}'
}
'''
        t = Template(templateText)
        context = {
            'dm' : self.declarativeModel,
            'level': levels[self.declarativeModel.level],
            'space': spaces[self.declarativeModel.cfgSpace]
        }
        return t.render(**context)




