from kgprim.core import Pose

class GeometricJacobian():
    '''
    classdocs
    '''


    def __init__(self, robotFrames, velocity):
        self.robot = robotFrames.robot
        self.velocity = velocity
        tgt = robotFrames.framesByName[ velocity.target.name ]
        ref = robotFrames.framesByName[ velocity.reference.name ]

        # Build the frames path from reference to target, and traverse it to
        # see which are the joints involved in this Jacobian. We also need to
        # determine the polarity for each joint: if the frames path (i.e. the
        # chain) goes across the joint in the successor-predecessor direction,
        # then the joint velocity S qdot basically has the opposite sign of what
        # is needed. We must mark this case.
        framesPath = robotFrames.path(ref, tgt)
        self.joints = []
        self.jointPoses = []
        self.polarities = []
        previousFrame = None
        for fr in framesPath :
            joint = robotFrames.joint(fr)
            if joint != None :
                self.joints.append( joint )
                self.jointPoses.append( Pose(target=fr, reference=ref))
                if previousFrame == robotFrames.framesByName[ self.robot.predecessor(joint).name ] :
                    self.polarities.append( 1 )
                else :
                    self.polarities.append( -1 )
            previousFrame = fr
        self.targetPose = Pose(target=tgt, reference=ref)

