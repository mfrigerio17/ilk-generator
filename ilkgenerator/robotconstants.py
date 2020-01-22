import numpy as np
from mako.template import Template

from ilkgenerator import codegenutils as tplutils

import ct.frommotions
import ct.numericbackend
from ct.models import PrimitiveCTransform


formatter = tplutils.FloatsFormatter(round_digits=6)

def oneTransformTable(ctransform):
    hm = ct.numericbackend.homogeneousCoordinates(ctransform)

    name = ctransform.rightFrame.name + "__" + ctransform.leftFrame.name
    p = tplutils.numericArrayToText( hm[0:3,3], formatter )
    R = tplutils.numericArrayToText( hm[0:3,0:3], formatter )

    name_inv = ctransform.leftFrame.name + "__" + ctransform.rightFrame.name
    p_inv    = tplutils.numericArrayToText( -hm[0:3,3], formatter )
    R_inv    = tplutils.numericArrayToText( np.transpose(hm[0:3,0:3]), formatter )
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
    transformsmodel = ct.frommotions.motionsToCoordinateTransforms(posesModel, ct.models.TransformPolarity.movedFrameOnTheRight )

    templateText = '''
return {
  poses = {
    % for mx in matrices :
        ${mx}
    % endfor
  }
}
'''
    template = Template(templateText)
    mxs = tplutils.commaSeparated(transformsmodel.transforms, oneTransformTable)
    return template.render( matrices=mxs )