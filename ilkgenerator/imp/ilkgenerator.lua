local tplmodule = require('template-text')

local function tpleval(text, env, opts)
  local options = opts or {}
  options.xtendStyle = true
  options.verbose = true
  return tplmodule.template_eval(text, env, options)
end

local function tpleval_failonerror(tpl, env, opts)
    local ok, text = tpleval(tpl, env, opts)
    if not ok then error(text) end
    return text
end

local function poseIdentifier(pose)
    return pose.target.name .. "__" .. pose.reference.name
end

local function velocityIdentifier(velocity)
    return "v__" .. velocity.tgt .. "__" .. velocity.ref
end

-- TODO s
-- * the constant poses should be only those referenced by this specific solver
-- * the same for the joint poses; these are actually more important. Depending
--   on the solver needs, the opposite-polarity transforms may be needed
-- both of the above can be achieved by reusing the optimal-composition solver,

local tpl = [[
return {
    solverid = '«solver_name»',
    solver_type = 'forward',
    robot_name = '«robot.name»',

    joints = {
@for i, joint in ipairs(robot.joints) do
        «joint.name» = { kind='«jTypeStr(joint)»', coordinate=«i» },
@end
    },

    poses = {
        constant = {
@for i, pose in ipairs(robot.constant_poses) do
            «poseID(pose)» = {},
@end
        },

        joint = {
@for joint, pose in pairs(robot.joint_poses) do
            «poseID(pose)» = {joint='«joint»', dir='a_x_b'},
@end
        },
    },

    joint_vel_twists = {
@for name, args in pairs(tape.joint_vel_twists) do
        «name» = { joint='«args.joint.name»', polarity=«args.polarity» },
@end
    },

    ops = {
@for i, op in ipairs(tape.ops) do
        «dumpOp(op)»,
@end
    },

    outputs = {
        v__link3__base = {otype='velocity', usersort=1 }
    },

}
]]

local function op_specs_to_text(op)
    local text = "{ op='" .. op.op .."', "
    for k,v in pairs(op) do
        if k ~= 'op' then
            text = text .. k .. "='" .. tostring(v) .. "', "
        end
    end
    text = text .. "}"
    return text
end


local names = require("names")
--local post_process = require("postp")

local function generate(robot_model, solver_tape)
    --post_process(solver_tape)
    local env = {
        robot = robot_model,
        tape  = solver_tape,
        jTypeStr = function(joint) return joint.kind.name end,
        poseID = names.poseIdentifier,
        dumpOp = op_specs_to_text,
        solver_name = 'foo',
    }

    return tpleval_failonerror(tpl, env)
end


return {
    generate = generate
}

