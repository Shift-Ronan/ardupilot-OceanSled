-- This script controls a TorqLink thruster via a CAN bus configured to 250 baud

-- Load CAN drivers, using the scripting protocol and with a buffer size of 8
local port_driver = CAN.get_device(8)

-- Init RC channels (Find definitions here: https://github.com/ArduPilot/ardupilot/blob/master/libraries/SRV_Channel/SRV_Channel.h)
local port_ch = 73 -- ThrottleLeft

local DA_MASK = 0x0000FF00
local SA_MASK = 0x000000FF
local PF_MASK = 0x00FF0000

function set_speed(s, driver)
    -- send speed control command

    msg = CANFrame()

    -- extended frame, priority 12, PGN 0xF003, and node ID 208 - (0x8CF003d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    msg:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF003")) << 8) | uint32_t(208))

    msg:data(0,255)
    msg:data(1, s) -- speed (0-250)
    msg:data(2,255)
    msg:data(3,255)
    msg:data(4,255)
    msg:data(5,255)
    msg:data(6,255)
    msg:data(7,255)
    -- sending 8 bytes of data
    msg:dlc(8)

    -- write the frame with a 10000us timeout
    driver:write_frame(msg, 10000)

	--gcs:send_text(0,"Sending speed")
end

function set_transmission(t, driver)
    -- set transmission to forward
    msg = CANFrame()

    -- extended frame, priority 12, PGN 0xF005, and node ID 208 - (0x8CF005d0)
    -- lua cannot handle numbers so large, so we have to use uint32_t userdata
    msg:id((uint32_t(1) << 31) | (uint32_t(12) << 24) | (uint32_t(tonumber("0xF005")) << 8) | uint32_t(208))

    msg:data(0,t) -- transmission gear (124-126)
    msg:data(1,255)
    msg:data(2,255)
    msg:data(3,255)
    msg:data(4,255)
    msg:data(5,255)
    msg:data(6,255)
    msg:data(7,255)
    -- sending 8 bytes of data
	msg:dlc(8)

    -- write the frame with a 10000us timeout
    driver:write_frame(msg, 10000)	
	--gcs:send_text(0,"Transmission set")
end

max_input = 1000
min_input = -1000
mid_input = 0 --math.floor((max_input+min_input)/2)
max_speed = 250 -- max speed for TQ thrusters
max_timeout = 10 -- max connection errors in a row
timeout = 0
start_time = 0
function update()
	-- Go back to idle if lost connection
	if not get_frame(port_driver) then
		timeout = timeout+1
		if timeout > max_timeout then
			timeout = max_timeout
			gcs:send_text(0, "Lost connection with TQ")
			speed = 0
			set_speed(speed, port_driver)
			set_transmission(125, port_driver)
			return check_ready()
		end
	else
		timeout = 0
	end
		
	-- Get direction and motor speed
	if arming:is_armed() then
		port_speed = SRV_Channels:get_output_scaled(port_ch)
	else
		port_speed = 0
	end

	port_dir = 125
	if port_speed ~= mid_input then
		port_dir = (port_speed > mid_input) and 126 or 124
	end
	-- Scale throttle to max for TQ
	port_scaled = math.floor(math.abs((((port_speed-mid_input)^2) / ((max_input-mid_input)^2)) * max_speed))
		
	-- Set direction and motor speed
	set_transmission(port_dir, port_driver)
	set_speed(port_scaled, port_driver)

	return update, 50
end

function get_frame(driver)
	if driver then
		frame = driver:read_frame()
		if frame then
			local id = tostring(frame:id())
			--gcs:send_text(5,string.format("CAN[%u] msg from " .. id .. ": %i, %i, %i, %i, %i, %i, %i, %i", 1, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
			return frame
		end
	end
	return false
end

function get_pgn(can_id)
	sa = SA_MASK & can_id
	pf = (PF_MASK & can_id) >> 16
	da = (DA_MASK & can_id) >> 8
	
	if pf >= 240 then
		pgn = pf * 256 + da
		da = 0xFF
	else
		pgn = pf * 256
	end
	return pgn
end

function check_ready()
	-- see if we got any frames
	frame = get_frame(port_driver)
	if frame then
		pgn = tostring(get_pgn(frame:id()))
		if pgn == "65299" then -- check if Torqeedo is ready by this message
			ready = frame:data(4) >> 7 -- Check first flag of thruster status bitmap

			if ready == 1 then
				gcs:send_text(0, "TQ Ready")
				return update, 2000 -- Need to wait for thruster to be ready
			end
		end
	end

	return check_ready, 10
end

return check_ready()