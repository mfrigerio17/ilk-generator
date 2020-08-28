local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

local opcodes = require('opcodes')
local names   = require('names')

local processors = {}
-- the default processor is the empty function
setmetatable( processors, {
        __index = function(tab, key) return function()
                logger.debug("No-op related to coordinate transforms, for opcode " .. key)
            end
        end
    }
)


processors[opcodes.vel_compose] = function(op, tape)
    -- the link whose velocity is being computed; we want to use its
    -- local coordinates for the computation
    local dest = op.res.tgt
    -- in the composition operands, look for a velocity of a different
    -- link; that is expected to be expressed with coordinates of such
    -- a link, which is therefore the frame we have to transform from
    local source = nil
    local toBeTransformed = nil
    if op.arg1.tgt ~= dest then
        source = op.arg1.tgt
        toBeTransformed = op.arg1
    elseif op.arg2.tgt ~= dest then
        source = op.arg2.tgt
        toBeTransformed = op.arg2
    end
    if source ~= nil then
        local ct = {type='twist', left_frame=dest, right_frame=source}
        local ct_name = names.ctIdentifier(ct)
        tape.ct[ct_name] = ct
        toBeTransformed.ct = ct_name
    end
end


local function coordinate_transforms_body(tape)
    for i, op in ipairs(tape.ops) do
        processors[op.op](op, tape)
    end
end

local function coordinate_transforms(tape, policy)
    local policy = policy or 'body-coordinates'
    if policy == 'body-coordinates' then
        coordinate_transforms_body(tape)
    else
        logger.error("Unrecognized policy. Only 'body-coordinates' is supported at the moment...")
    end
end


return {
    annotate_with_ctransforms = coordinate_transforms
}