local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

local opcodes = require('opcodes')


local function coordinate_transforms_body(tape)
    for i, op in ipairs(tape.ops) do
        if op.op==opcodes.vel_compose then
            -- the link whose velocity is being computed; we want to use its
            -- local coordinates for the computation
            local dest = op.res.tgt
            -- in the composition operands, look for a velocity of a different
            -- link; that is expected to be expressed with coordinates of such
            -- a link, which is therefore the frame we have to transform from
            local source = nil
            if op.arg1.tgt ~= dest then
                source = op.arg1.tgt
            elseif op.arg2.tgt ~= dest then
                source = op.arg2.tgt
            end
            if source ~= nil then
                table.insert( tape.ct, {type='twist', left_frame=dest, right_frame=source} )
            end
        end
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