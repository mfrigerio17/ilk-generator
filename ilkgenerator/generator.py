'''
Created on Nov 22, 2018

@author: marco
'''

from mako.template import Template
from collections import namedtuple

from ilkgenerator import query
from ilkgenerator import codegenutils

from gr import core as gr
from robmodel import frames

def poseIdentifier(pose):
    return pose.target.name + "__" + pose.reference.name

def velocityIdentifier(velocity):
    return "v__"+velocity.target.name+"__"+velocity.reference.name

def gJacobianIdentifier(gjac):
    return "J_" + gjac.velocity.target.name + "_" + gjac.velocity.reference.name


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



BlockSpec = namedtuple('BlockSpec', ['lineTemplate', 'singleItemName', 'context'])

class SweepingSolverGenerator():
    def __init__(self, solvermodel):
        poseComposes = []
        for composition in solvermodel.poseComposes :
            poseComposes.extend( composition.asSequenceOfBinaryCompositions() )

        self.explicitJointVelocities = solvermodel.jointVelocitiesExplicit
        self.velComposes  = solvermodel.velBinaryComposes
        self.poseComposes = poseComposes
        self.solverModel  = solvermodel

        def outputIndex(self):
            total = sum( [len(block) for block in self.solverModel.output.values() ] )
            for i in range(0,total) :
                yield i+1
        self.outputVarIndexGenerator = outputIndex(self)


    def jointNum(self, joint):
        '''The integer coordinate for the given joint.
        The joint coordinate for 0-based sequences should be the joint code
        in the robot model -1. This is because the regular numbering for joints
        start with 1, which is the joint connecting the base (#0) with the
        link #1.'''
        return self.solverModel.robot.jointNum(joint)-1

    def commaSepLines(self, sequence, genSpec):
        generator = codegenutils.singleItemTemplateRenderer(genSpec.lineTemplate, genSpec.singleItemName, genSpec.context)
        return codegenutils.commaSeparated(sequence, generator)

    def block_modelJoints(self):
        bspec = BlockSpec(
            lineTemplate   = '''${joint.name} = { kind='${typeStr(joint)}', coordinate=${jnum(joint)} }''',
            singleItemName = 'joint',
            context = {'typeStr': jointTypeStr, 'jnum': self.jointNum}
        )
        return self.commaSepLines(self.solverModel.robot.joints.values(), bspec)


    def block_constantPoses(self):
        bspec = BlockSpec(
            lineTemplate = '''${toID(pose)}={}''',
            singleItemName = 'pose',
            context = {'toID' : poseIdentifier}
        )
        return self.commaSepLines(self.solverModel.constPoses, bspec)


    def block_jointPoses(self):
        bspec = BlockSpec(
            lineTemplate = '''${toID(pose)} = { joint='${pose.joint.name}', dir='${dirTag(pose)}' }''',
            singleItemName = 'pose',
            context = {'toID': poseIdentifier, 'dirTag': directionTag}
        )
        return self.commaSepLines(self.solverModel.jointPoses, bspec)


    def block_jointVelocityTwists(self):
        def singleLine(jvel):
            tpl = ""
            poseid = ""
            if jvel.polarity == -1 :
                refF = self.solverModel.robotFrames.framesByName[jvel.vel.target.name]
                tgtF = self.solverModel.robotFrames.framesByName[jvel.joint.name]
                pose = gr.Pose(target=tgtF, reference=refF)
                poseid = poseIdentifier(pose)
                tpl = "${velid} = { joint='${joint.name}', polarity=-1, ctransform='${pose}' }"
            else :
                tpl = "${velid} = { joint='${joint.name}', polarity=1 }"

            context = {'velid' : velocityIdentifier(jvel.vel),
                       'joint' : jvel.joint,
                       'pose'  : poseid }
            return Template(tpl).render(**context)

        bspec = BlockSpec(
            lineTemplate = "${line(jvel)}",
            singleItemName = "jvel",
            context = {'line' : singleLine}
        )
        return self.commaSepLines(self.solverModel.jointVelocities.values(), bspec)


    def block_jacobians(self):
        def oneJac(J):
            Jid = gJacobianIdentifier(J)

            column_op = BlockSpec(
                lineTemplate = '''{ op='GJac-col', joint='${J.joints[j].name}', jac='${gjac_id}', col=${jnum(J.joints[j])}, joint_pose='${poseID(J.jointPoses[j])}', polarity=${J.polarities[j]} }''',
                singleItemName = "j",
                context = { "gjac_id" : Jid,
                            "jnum"    : self.jointNum,
                            "poseID"  : poseIdentifier,
                            "J"       : J
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

        bspec = BlockSpec(
            lineTemplate = "${block(J)}",
            singleItemName = "J",
            context = {'block' : oneJac}
        )
        return self.commaSepLines(self.solverModel.geometricJacobians, bspec)


    def block_poseComposes(self):
        bspec = BlockSpec(
            lineTemplate = '''{ op='pose-compose', arg1='${toID(c.arg1)}', arg2='${toID(c.arg2)}', res='${toID(c.result)}' }''',
            singleItemName = "c",
            context = {'toID' : poseIdentifier}
        )
        return self.commaSepLines(self.poseComposes, bspec)


    def block_velocityCompose(self):
        bspec = BlockSpec(
            lineTemplate = '''{ op='vel-compose', arg1='${toID(c.arg1)}', arg2='${toID(c.arg2)}', pose='${poseID(c.pose)}', res='${toID(c.result)}' }''',
            singleItemName = 'c',
            context = {'toID' : velocityIdentifier, 'poseID' : poseIdentifier}
        )
        return self.commaSepLines(self.velComposes, bspec)


    def block_explicitJointVelTwists(self):
        bspec = BlockSpec(
            lineTemplate = '''{ op='joint-vel-twist', arg='${toID(jv)}' }''',
            singleItemName = 'jv',
            context = {'toID' : velocityIdentifier}
            )
        return self.commaSepLines(self.explicitJointVelocities, bspec)


    def block_outputPoses(self):
        bspec = BlockSpec(
            lineTemplate = '''${toID(pose)} = {otype='pose', usersort=${next(oindex)} }''',
            singleItemName = 'pose',
            context = {'toID': poseIdentifier, 'oindex': self.outputVarIndexGenerator}
            )
        return self.commaSepLines(self.solverModel.output['pose'], bspec)


    def block_outputVelocities(self):
        bspec = BlockSpec(
            lineTemplate = '''${toID(vel)} = {otype='velocity', usersort=${next(oindex)} }''',
            singleItemName = 'vel',
            context = {'toID' : velocityIdentifier, 'oindex': self.outputVarIndexGenerator}
        )
        return self.commaSepLines(self.solverModel.output['velocity'], bspec)


    def block_outputJacobians(self):
        bspec = BlockSpec(
            lineTemplate = "${toID(gjac)} = {otype='jacobian', usersort=${next(oindex)} }",
            singleItemName = 'gjac',
            context = {'toID' : gJacobianIdentifier, 'oindex': self.outputVarIndexGenerator}
        )
        return self.commaSepLines(self.solverModel.output['jacobian'], bspec)


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
    joints = {
    % for jointspec in this.block_modelJoints() :
        ${jointspec}
    % endfor
    },
    poses = {
        constant = {
        % for pose in this.block_constantPoses() :
            ${pose}
        % endfor
        },
        joint = {
        % for pose in this.block_jointPoses() :
            ${pose}
        % endfor
        }
    },
    joint_vel_twists = {
        % for jvel in this.block_jointVelocityTwists() :
        ${jvel}
        % endfor
    },
    ops = {
    % for composition in this.block_poseComposes() :
        ${composition}
    % endfor
    ${ next(ops_separator) }

    % for jointVel in this.block_explicitJointVelTwists() :
        ${jointVel}
    % endfor
    ${ next(ops_separator) }

    % for velcompose in this.block_velocityCompose() :
        ${velcompose}
    % endfor
    ${ next(ops_separator) }

    % for gjac in this.block_jacobians() :
${gjac}
    % endfor
    },

    outputs = {
    % for p in this.block_outputPoses() :
        ${p}
    % endfor
    ${ next(out_separator) }

    % for v in this.block_outputVelocities() :
        ${v}
    % endfor
    ${ next(out_separator) }

    % for J in this.block_outputJacobians() :
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




