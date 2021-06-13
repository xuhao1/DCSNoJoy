local log_file = nil

package.path  = package.path..";.\\LuaSocket\\?.lua"
package.cpath = package.cpath..";.\\LuaSocket\\?.dll"
socket = require("socket")
-- host = host or "localhost"
-- port = port or 27015
host = "127.0.0.1"
port = 27015

function LuaExportStart()
    log_file = io.open("C:/Users/xuhao/Saved Games/DCS/Logs/EasyControl.log", "w")
    log.write("EasyControl.EXPORT", log.INFO, "initializing DCS Easy Control")
end

function LuaExportBeforeNextFrame()
end

function LuaExportAfterNextFrame()
end

function LuaExportStop()
    
    if log_file then
        log_file:close()
        log_file = nil
    end	

    if connectSoc then
        -- socket.try(connectSoc:send("quit")) -- to close the listener socket
        connectSoc:close()
    end
end

function LuaExportActivityNextEvent(t)
    if connectSoc==nil then
        -- log_file:write("try to open socket\n")
        log.write("EasyControl.EXPORT", log.INFO, "try to open socket")
        
        connectSoc = assert(socket:udp())
        assert(connectSoc:setoption('broadcast', true))
        assert(connectSoc:setoption('dontroute', true))   -- do we need this?

        if connectSoc then
            log.write("EasyControl.EXPORT", log.INFO, "socket ok")
        else
            return t+0.5
        end
    end

    local myData = LoGetSelfData()
    if (myData) then
        local altBar = LoGetAltitudeAboveSeaLevel()
        local altRad = LoGetAltitudeAboveGroundLevel()
        local pitch, roll, yaw = myData.Pitch, myData.Bank, myData.Heading
        -- local pitch, roll, yaw = LoGetADIPitchBankYaw()
        local pos = myData.Position
        local _type = myData.Type
        local aoa = LoGetAngleOfAttack()
        local tas = LoGetTrueAirSpeed()
        local omega = LoGetAngularVelocity()
        local cam_pos = LoGetCameraPosition()
        log.write("EasyControl.EXPORT", log.INFO, string.format(
            "type %d %d %d %d name %s cam Rxx=%.3f,Ryx=%.5f,Rzx=%.5f,p %.1f %.1f %.1f aircraft %.1f %.1f %.1f", 
            _type.level1, _type.level2, _type.level3, _type.level4, myData.Name,
            cam_pos.x.x, cam_pos.y.x, cam_pos.z.x, cam_pos.p.x, cam_pos.p.y, cam_pos.p.z,
            pos.x, pos.y, pos.z))
        
        local _datalog = string.format(
            "name=%s altBar=%.3f x=%.5f y=%.5f z=%.5f pitch=%.5f roll=%.5f yaw=%.5f yawrate=%.5f pitchrate=%.5f rollrate=%.5f tas=%.3f aoa=%.5f\n", 
            myData.Name, altBar, pos.x, pos.y, pos.z, pitch, roll, yaw, omega.y, omega.z, omega.x, tas, aoa)
        -- local _datalog = string.format(
        --         "altBar=%.3f,pitch=%.5f,roll=%.5f,yaw=%.5f,yawrate=%.5f,pitchrate=%.5f,rollrate=%.5f,tas=%.3f,aoa=%.5f\n", 
        --         altBar, pitch, roll, yaw, omega.y, omega.z, omega.x, tas, aoa)
    
        socket.try(connectSoc:sendto(_datalog, host, port))
        log_file:write(_datalog)

    else
    end
    return t+0.01
end

-- Index 00 = LoGetAccelerationUnits().x = Lateral acceleration (G)
-- Index 01 = LoGetAccelerationUnits().z = Longitudinal acceleration (G)
-- Index 02 = LoGetAccelerationUnits().y = Vertical acceleration (G)
-- Index 03 = LoGetVectorVelocity().x = Lateral speed (m/s)
-- Index 04 = LoGetVectorVelocity().z = Longitudinal speed (m/s)
-- Index 05 = LoGetVectorVelocity().y = Vertical speed (m/s)
-- Index 06 = LoGetAngularVelocity().z = Rotation speed around z (pitchrate in rad/s) body frame
-- Index 07 = LoGetAngularVelocity().y = Rotation speed around y (yawrate in rad/s) body frame
-- Index 08 = LoGetAngularVelocity().x = Rotation speed around x (rollrate in rad/s) body frame 
-- Index 09 = LoGetADIPitchBankYaw(0) = Yaw position (rad)
-- Index 10 = LoGetADIPitchBankYaw(1) = Roll position (rad)
-- Index 11 = LoGetADIPitchBankYaw(2) = Pitch position (rad)
-- Index 12 = LoGetTrueAirSpeed() = Air speed (m/s)
-- Index 13 = LoGetAircraftDrawArgumentValue(1) = Front/Rear landing gear (0 to 1)?
-- Index 14 = LoGetAircraftDrawArgumentValue(2) = Turning landing gear (0 to 1)?
-- Index 15 = LoGetAircraftDrawArgumentValue(4) = Left landing gear (0 to 1)?
-- Index 16 = LoGetAircraftDrawArgumentValue(6) = Right landing gear (0 to 1)?
-- Index 17 = LoGetAltitudeAboveGroundLevel() = Vertical position relative to ground (m)
-- Index 17 = LoGetModelTime() = To track frame sequence (not used right now, in seconds)

-- local cam_pos = LoGetCameraPosition() x y z is a rotation matrix
