function dbg(tab) for k,v in pairs(tab) do print(k,v) end end

local logger = require('log').new(
  "debug",
  require('log.writer.console.color').new()
)
local opcodes = require("opcodes")
local names = require("names")


local function velocity_meta(tape)
    local function id(v) return names.velocityIdentifier(v) end

    local mt_velocity = {
        __add = nil,
        __eq  = function(lhs, rhs)
            --print( id(lhs), id(rhs) )
            --print( lhs.tgt==rhs.tgt , lhs.ref.name==rhs.ref.name, type(lhs.tgt), type(rhs.tgt), type(lhs.ref), type(rhs.ref) )
            return lhs.tgt.name==rhs.tgt.name and lhs.ref.name==rhs.ref.name
        end
    }

    local function velocity_add(lhs, rhs)
        -- TODO target/reference of the composition should be figured out by
        -- looking at the addends
        local ret = { tgt=rhs.tgt,  ref=lhs.ref }
        setmetatable(ret, mt_velocity)
        --print(lhs.tgt,lhs.ref,rhs.tgt,rhs.ref,ret.tgt,ret.ref, ret.ref==rhs.ref)
        if (ret==lhs) or (ret==rhs) then
            logger.debug("Skipping no-op velocity composition of '" .. id(lhs) .. "' and '" .. id(rhs) .. "'")
        else
            table.insert( tape.ops, { op=opcodes.vel_compose, arg1=lhs, arg2=rhs, res=ret} )
        end
        return ret
    end

    mt_velocity.__add = velocity_add

    local mt = {
        -- make sure to overload addition for all the elements added to the table
        __newindex = function(t, key, val)
            if val ~= nil then
                setmetatable( val, mt_velocity )
                if key.name ~= val.tgt.name then
                    logger.warning("Assigning velocity of '" .. val.tgt.name .. "' to '" .. key .. "'")
                end
                rawset(t, key.name, val)
                --print("__newindex", t, key.name, val, t[key], t[key.name])
            end
        end,
        __index = function(tab, key)
            if key == nil then return nil end
            --print("trying table ", tab, "key ", key.name)
            return tab[key.name]
        end
    }

    local function new( args )
        local v = {tgt=args.tgt, ref=args.ref}
        setmetatable(v, mt_velocity)
        return v
    end

    local function dictionary()
        local v = {}
        setmetatable(v, mt)
        return v
    end

    return{
        new = new,
        dictionary = dictionary,
    }
end





local function environment(robot)

    local tape = {
        ct = {},
        joint_vel_twists = {},
        ops = {}
    }

    local parent   = robot.parent
    local supportJ = robot.supportJ
    local predecessor = robot.predecessor
    local successor   = robot.successor

    local vmeta = velocity_meta(tape)
    local v = vmeta.dictionary()

    local function vJoint(joint)
        local ret = vmeta.new( { tgt = successor(joint), ref = predecessor(joint) } )
        local v_name = names.velocityIdentifier(ret)
        tape.joint_vel_twists[v_name] = { joint=joint, polarity=1 } --TODO polarity
        table.insert( tape.ops, { op=opcodes.joint_vel_twist, arg=v_name} )

        return ret
    end

    local function zeroV(link)
        v[link] = vmeta.new( { tgt=link, ref=link } )
    end

    robot.bodies_outward_sweep = function() return ipairs(robot.links) end

    local env = {
        robot = robot,
        v = v,
        parent   = parent,
        supportJ = supportJ,
        vJoint   = vJoint,
        zeroV    = zeroV,

        print = print, -- for debugging inside a DPC source
        type = type,
        pairs = pairs
    }
    return env, tape
end


local function interpret(solver_code, env)
    local env = env or {}
    local f, msg = load(solver_code, nil, 't', env)
    --print(f, msg)
    if f==nil then
        return false, {loadError=true, msg=msg}
    end
    pcall(f)
    --for k,v in pairs(env) do print(k,v) end
    env.solver()
    return true, {msg="success"}
end


local function dump(entry, indent)
    indent = indent or ""
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

local function tape_dump(tape)
    print("")
    print("Recorded tape dump:")
    print("")
    print("Joint velocities:")
    print( dump( tape.joint_vel_twists ) )
    print("")
    print("Operations:")
    print( dump(tape.ops ))
    print("")
    print("Coordinate transforms:")
    print( dump(tape.ct))
end

return {
    environment = environment,
    interpret = interpret,
    tape_dump = tape_dump,
}
