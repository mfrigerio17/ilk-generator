local opcodes = require("opcodes")
local names   = require("names")

local function add_pose_composes(tape, pose_composes)
    tape.pose_compose_ops = {}
    for pc in python.iter(pose_composes) do
        table.insert(tape.pose_compose_ops,
            { op = opcodes.pose_compose,
              arg1 = names.poseIdentifier( pc.arg1 ),
              arg2 = names.poseIdentifier( pc.arg2 ),
              res = names.poseIdentifier( pc.result ),
            })
    end

end


return {
    add_pose_composes = add_pose_composes,
}