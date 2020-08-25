local logger = require("log").new(
    "warning", require("log.writer.console.color").new()
)
local interpreter = require("interpret")
local ct = require("ct")
local post_process = require("postp")
local ilkgen = require("ilkgenerator")


local function main(robot_model, dpc_program)

    -- get the interpretation environment and the recording tape
    local env, tape = interpreter.environment(robot_model)

    -- actually interpret the pseudo-code
    local ok, err = interpreter.interpret(dpc_program, env)
    if not ok then
        logger.error("Failed to interpret the DPC program:")
        logger.error(err.msg)
    end

    -- augment the tape with the required coordinate transforms
    ct.annotate_with_ctransforms(tape, 'body-coordinates')

    -- post-process the tape to prepare it for the ILK-generation
    post_process(tape)

    -- generate the ILK solver model
    local ilk = ilkgen.generate(robot_model, tape)

    --interpreter.tape_dump(tape)

    return ilk
end

return {
    main = main
}