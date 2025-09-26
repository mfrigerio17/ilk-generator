import numpy as np
from mako.template import Template

from ilkgenerator import codegenutils as tplutils

import kgprim.ct.frommotions as mot2ct
import kgprim.ct.repr.mxrepr as mxrepr
from kgprim.ct.models import PrimitiveCTransform
from kgprim.ct.models import TransformPolarity

from robmodel.connectivity import JointKind

formatter = tplutils.FloatsFormatter(round_digits=6)

def _tformIdentifier(targetFrame, relativeToFrame):
    return targetFrame.name + "__" + relativeToFrame.name

def oneTransformTable(poseSpec):
    ct1 = mot2ct.toCoordinateTransform(poseSpec, polarity=TransformPolarity.movedFrameOnTheRight)
    ct2 = mot2ct.toCoordinateTransform(poseSpec, polarity=TransformPolarity.movedFrameOnTheLeft)

    hm = mxrepr.hCoordinatesNumeric(ct1)
    name = _tformIdentifier(targetFrame=ct1.rightFrame, relativeToFrame=ct1.leftFrame)
    p = tplutils.numericArrayToText( hm[0:3,3], formatter )
    R = tplutils.numericArrayToText( hm[0:3,0:3], formatter )

    hm_inv = mxrepr.hCoordinatesNumeric(ct2)
    name_inv = _tformIdentifier(targetFrame=ct2.rightFrame, relativeToFrame=ct2.leftFrame)
    p_inv    = tplutils.numericArrayToText( hm_inv[0:3,3], formatter )
    R_inv    = tplutils.numericArrayToText( hm_inv[0:3,0:3], formatter )
    templateText ='''
${name} = {
    p = {${p[0]}, ${p[1]}, ${p[2]}},
    r = {${R[0,0]},${R[0,1]},${R[0,2]},
         ${R[1,0]},${R[1,1]},${R[1,2]},
         ${R[2,0]},${R[2,1]},${R[2,2]}}
},
${name_inv} = {
    p = {${p_inv[0]}, ${p_inv[1]}, ${p_inv[2]}},
    r = {${R_inv[0,0]},${R_inv[0,1]},${R_inv[0,2]},
         ${R_inv[1,0]},${R_inv[1,1]},${R_inv[1,2]},
         ${R_inv[2,0]},${R_inv[2,1]},${R_inv[2,2]}}
}'''
    return Template(templateText).render( p=p, R=R, name=name, name_inv=name_inv, p_inv=p_inv, R_inv=R_inv )


def asLuaTable(robotGeometryModel):
    posesModel      = robotGeometryModel.posesModel

    fixed_joints = []
    connectModel = robotGeometryModel.connectivityModel
    framesModel  = robotGeometryModel.framesModel
    for joint in connectModel.joints.values() :
        if joint.kind == JointKind.fixed :
            jFrame = framesModel.byJoint[joint]
            lFrame = framesModel.byLink[connectModel.successor(joint)]
            id1 = _tformIdentifier(targetFrame=jFrame, relativeToFrame=lFrame)
            id2 = _tformIdentifier(targetFrame=lFrame, relativeToFrame=jFrame)
            fixed_joints.append(id1)
            fixed_joints.append(id2)

    templateText = '''
return {
  poses = {
    % for mx in matrices :
        ${mx}
    % endfor
  ,
    % for j in fixed_joints :
${j} = '_identity_',
    % endfor
  }
}
'''
    template = Template(templateText)
    mxs = tplutils.commaSeparated(posesModel.poses, oneTransformTable)
    return template.render( matrices=mxs, fixed_joints=fixed_joints )
