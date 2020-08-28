local function poseIdentifier(pose)
    return pose.target.name .. "__" .. pose.reference.name
end

local function velocityIdentifier(velocity)
    return "v__" .. velocity.tgt.name .. "__" .. velocity.ref.name
end

local function ctIdentifier(ct)
    local symbol = 'X'
    if ct.type == "twist" then symbol="XM" end
    return ct.left_frame.name .. "_" .. symbol .. "_" .. ct.right_frame.name
end


return {
    poseIdentifier = poseIdentifier,
    velocityIdentifier = velocityIdentifier,
    ctIdentifier = ctIdentifier,
}