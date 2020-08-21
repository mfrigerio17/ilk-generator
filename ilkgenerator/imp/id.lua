local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

--print(_G)
--print(_ENV)

local a = {}
local v = {}
local f = {}
local tau = {}



local robot = {
    name = 'dummy',
    bodies = { 'link1', 'link2', 'link3'},
    joints = { 'j1', 'j2', 'j3'},
    parent = {
        link1 = 'base',
        link2 = 'link1',
        link3 = 'link2',
    },
    sjoint = {
        link1 = 'j1',
        link2 = 'j2',
        link3 = 'j3',
    },
    predecessor = {
        j1 = 'base', j2 = 'link1', j3 = 'link2'
    },
    successor = {
        j1 = 'link1', j2 = 'link2', j3 = 'link3'
    }
}

local tape = {
    ct = {},
    joint_vel_twists = {},
    ops = {}
}

robot.bodies.outward = function() return ipairs(robot.bodies) end

local function parent(link)   return robot.parent[link] end
local function supportJ(link) return robot.sjoint[link] end
local function predecessor(joint) return robot.predecessor[joint] end
local function successor(joint)   return robot.successor[joint] end

local function vJoint(joint)
    table.insert( tape.joint_vel_twists, { joint=joint, polarity=1 } ) --TODO polarity
    table.insert( tape.ops, { op='joint-vel-twist', joint=joint} )
    return { tgt = successor(joint), ref = predecessor(joint) }
end

local function aJoint(joint)
    return { val = 'a_' .. joint }
end

local function a_bias(joint)
    return { val = "v_i x " .. vJoint(joint).val }
end






local mt_add = {
    __add = nil
}

local function velocity_add(lhs, rhs)
    table.insert( tape.ops, { op='vel-compose', arg1=lhs, arg2=rhs} )
    table.insert( tape.ct, {type='twist', left_frame=rhs.tgt, right_frame=lhs.ref} )
    -- TODO target/reference of the composition should be figured out by
    -- looking at the addends
    local ret = { tgt=rhs.tgt,  ref=lhs.ref }
    setmetatable(ret, mt_add)
    return ret
end

mt_add.__add = velocity_add

local mt = {
    -- make sure to overload addition for all the elements added to the table
    __newindex = function(t, key, val)
        setmetatable( val, mt_add )
        --print("assigning key ", key, "in table", t)
        rawset(t, key, val)
    end
}
setmetatable(v, mt)


local function v_zero(link)
    v[link] =  { tgt=link, ref=link }
end

local locals = {
    _G = _G,  -- don't forget ;) - to be able to re-get _G as a global
    
    v_zero = v_zero
}
local _ENV = locals

--print(_G)
--print(_ENV)

function a_solver(robot, q, qd, qdd)

    v_zero('base')

    for i, link in robot.bodies.outward() do

        joint = supportJ(link)
    
        v_J = vJoint( joint )
        dad = parent(link)
        v[link] = v[dad] + v_J

        --a_J = aJoint( joint )
        --a[link] = a[parent(link)] + a_J + a_bias( joint )
        --f[link] = inertia(link) * a[link] + f_bias(link)
    end
end

--for link in robot.bodies.inward() do
--    joint = supportJ(link)
--    tau[joint] = joint_project( f[link ] )
--    if parent(link) ~= robot.base then
--        f[ parent(link) ] = f[ parent(link) ] + f[link]
--    end
--end

a_solver(robot)


_ENV = _G -- restore standard

--print("")
--for k,v in pairs(locals) do
--    print(k,v)
--end


print("")

local function dump(entry, indent)
    indent = indent or '\t'
    local text = ""
    for k,v in pairs(entry) do
        if type(v)=='table' then
            text = text .. k .. ' :\n' .. dump(v, indent..'\t') .. '\n' .. indent
        else
            text = text .. k .. ' : ' .. tostring(v) .. '    '
        end
    end
    return indent .. text
end

print("Recorded tape dump:")
print("")
print("Joint velocities:")
print( dump( tape.joint_vel_twists, '\t' ) )
print("")
print("Operations:")
print( dump(tape.ops, '\t' ))
--for k,v in pairs(registry) do
--    print(k, dump(v, '\t'))
--end
