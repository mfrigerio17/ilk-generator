local opcodes = require("opcodes")
local names   = require("names")

local processors = {}
setmetatable( processors,
{
    __index = function(tab, key) return function() end end
}
)

local function vel_compose(op)
    local ret = op
    ret.arg1 = names.velocityIdentifier( op.arg1 )
    ret.arg2 = names.velocityIdentifier( op.arg2 )
    ret.res  = names.velocityIdentifier( op.res )
end

processors[opcodes.vel_compose] = vel_compose



local function post_process(tape)
    for i, op in ipairs(tape.ops) do
        processors[op.op](op)
    end
    return tape
end

return post_process