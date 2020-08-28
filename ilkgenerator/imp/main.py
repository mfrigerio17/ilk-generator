import logging

from kgprim.core import Pose

import ilkgenerator.imp.luabridge as luabridge
import ilkgenerator.optcompose  as optcompose
import ilkgenerator.posecompose as posecompose

logger = logging.getLogger(__name__)

gen = luabridge.load_module('ilkgenerator.lua')
exe = luabridge.load_module('interpret.lua')
ct  = luabridge.load_module('ct.lua')
pp  = luabridge.load_module('postp.lua')
posec  = luabridge.load_module('posecompose.lua')


def main(robot, dpc_program):
    luamodel = luabridge.lua_friendly_robot(robot)

    # get the interpretation environment and the recording tape
    env, tape = exe.environment(luamodel)

    # actually interpret the pseudo-code
    ok, err = exe.interpret(dpc_program, env)
    if not ok :
        logger.error("Failed to interpret the DPC program:")
        logger.error(err.msg)

    # augment the tape with the required coordinate transforms
    ct.annotate_with_ctransforms(tape, 'body-coordinates')

    poseCompositionPaths = []
    for name,tf in tape.ct.items() :
        # map from Lua objects to actual Python robot frames
        lxf = robot.frames.framesByName[tf.left_frame.name]
        rxf = robot.frames.framesByName[tf.right_frame.name]

        pose = Pose(target=lxf, reference=rxf)
        pose_path = posecompose.posePath(robot_frames=robot.frames, givenpose=pose )
        poseCompositionPaths.append(pose_path)

    poseComposes = optcompose.allComposes( poseCompositionPaths )
    bin_composes = []
    for composition in poseComposes :
        bin_composes.extend( composition.asSequenceOfBinaryCompositions() )

    posec.add_pose_composes(tape, bin_composes)

    # post-process the tape to prepare it for the ILK-generation
    pp.post_process(tape)

    # generate the ILK solver model
    ilk = gen.generate(luamodel, tape)
    return ilk