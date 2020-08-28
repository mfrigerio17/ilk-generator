import kgprim.core
from kgprim.core import Pose
from robmodel.frames import FrameRelationKind

import ilkgenerator.optcompose as optcompose
from ilkgenerator.optcompose import HomogenoeusComposable

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
            current = kgprim.core.poseCompose(current, ot.originalPose)
        return _ComposablePose(current)


def posePath(robot_frames, givenpose):
    '''
    The sequence of distance-1 poses equivalent to the given pose.

    Distance-1 poses involve adjacent frames, i.e. their value is given in
    the robot model.
    The returned sequence is constructed from the shortest path connecting
    the target frame and the reference frame of the original given pose.

    The returned sequence is in fact an `ilkgenerator.optcompose.Path` object.
    '''
    composablesList = []
    graphPath = robot_frames.path(givenpose.target, givenpose.reference)
    tgt = givenpose.target
    for ref in graphPath[1:] :
        pose = _ComposablePose( Pose(tgt, ref) )
        if robot_frames.kind(tgt, ref) is FrameRelationKind.acrossJoint :
            pose.joint = robot_frames.joint(tgt, ref)

        composablesList.append( pose )
        tgt = ref

    return optcompose.Path(composablesList)

