import os
import lupa

lua_runtime = lupa.LuaRuntime(unpack_returned_tuples=True)


def lua_friendly_robot(robot) :
    links  = lua_runtime.table_from(robot.tree.codeToLink)
    joints = lua_runtime.table_from(robot.tree.codeToJoint)

    constant_poses = [pspec.pose for pspec in robot.kinematics.geomPosesByJoint.values()]
    joint_poses = { joint.name:pspec.pose for joint, pspec in robot.kinematics.jointPosesByJoint.items() }

    luarobot = {
        'pyrob' : robot,
        'name' : robot.name,
        'base' : robot.tree.base,
        'treeutils' : robot.treeutils,
        'fb'   : robot.fb,
        "joints" : joints,
        "links"  : links,
        "constant_poses" : lua_runtime.table_from(constant_poses),
        "joint_poses" : lua_runtime.table_from(joint_poses),
        "parent"   : robot.treeutils.parent,
        "supportJ" : robot.treeutils.supportingJoint,
        "predecessor" : robot.tree.predecessor,
        "successor"   : robot.tree.successor,
    }
    return luarobot

def load_module(path):
    luaCodeSrc = open( os.path.join( os.path.dirname(__file__), path), "r")
    ret = lua_runtime.execute(luaCodeSrc.read())
    luaCodeSrc.close()
    return ret


load_module("opcodes.lua")
load_module("names.lua")
load_module("ct.lua")
