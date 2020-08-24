local function poseIdentifier(pose)
    return pose.target.name .. "__" .. pose.reference.name
end

local function velocityIdentifier(velocity)
    return "v__" .. velocity.tgt.name .. "__" .. velocity.ref.name
end

return {
poseIdentifier = poseIdentifier,
velocityIdentifier = velocityIdentifier
}