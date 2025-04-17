import robmodel.connectivity

def isSupportedTypeAndNonFixed(joint):
    '''
    Tell whether the given joint is supported by this tool and relevant for the
    solvers models.

    In principle we support anything in the metamodel of the robot-model-tools
    package (`robmodel.connectivity`), which at the moment accounts only for
    prismatic and revolute joints, except the fixed type.
    '''
    return joint.kind is not robmodel.connectivity.JointKind.fixed
