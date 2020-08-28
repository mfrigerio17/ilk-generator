local opcodes = require("opcodes")
local names   = require("names")

local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

--- At the moment, this module contains the post processing of the generated
-- ILK operations in order to make them compatible with the current version of
-- the ILK compiler. This means most of the time to hack in some policies/shortcuts
-- due to the limitations of the ILK compiler


local processors = {}
setmetatable( processors,
{
    __index = function(tab, key) return function() end end
}
)

local function vel_compose(op, tape)
    local function ct_to_pose_name(ct_name)
    -- to deal with another hack in the ILK compiler: the required coordinate
    -- transform is just referred to with the name of the corresponding relative
    -- pose
        local ct = tape.ct[ct_name]
        if ct == nil then
            logger.warning("Coordinate transform '" .. ct_name .. "' not found")
            return ""
        end
        return names.poseIdentifier({target=ct.left_frame, reference=ct.right_frame})
    end

    -- the ILK compiler always expect the velocity whose target is the same as
    -- the resulting velocity, to be the first argument of the composition...

    if op.arg1.ct ~= nil then
        op.pose = ct_to_pose_name(op.arg1.ct)
        -- a hacky shortcut to indicate the coordinate transform, required by the ILK compiler
    end
    if op.arg2.ct ~= nil then
        op.pose = ct_to_pose_name(op.arg2.ct)
    end
    local v1 = names.velocityIdentifier( op.arg1 )
    local v2 = names.velocityIdentifier( op.arg2 )
    if op.arg1.tgt == op.res.tgt then
        op.arg1 = v1
        op.arg2 = v2
    else
        op.arg1 = v2
        op.arg2 = v1
    end
    op.res  = names.velocityIdentifier( op.res )
end

processors[opcodes.vel_compose] = vel_compose



local function post_process(tape)
    for i, op in ipairs(tape.ops) do
        processors[op.op](op, tape)
    end
    return tape
end

return {
    post_process = post_process,
}