-- This script controls two TorqLink thrusters via separate CAN buses configured to 250 baud

-- Load CAN drivers, using the scripting protocol and with a buffer size of 8
local port_driver = CAN.get_device(8)
local stbd_driver = CAN.get_device2(8)

-- Init RC channels (Find definitions here: https://github.com/ArduPilot/ardupilot/blob/master/libraries/SRV_Channel/SRV_Channel.h)
local port_ch = 73 -- ThrottleLeft
local stbd_ch = 74 -- ThrottleRight

-- TorqLink PGN to get ready status from
local TQ_STATUS_PGN = "65299"

-- Masks for CAN ID parser
local DA_MASK = 0x0000FF00
local SA_MASK = 0x000000FF
local PF_MASK = 0x00FF0000



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

function get_frame(dnum, driver)
	if driver then
		frame = driver:read_frame()
		if frame then
			local id = tostring(frame:id())
--			if tostring(get_pgn(frame:id())) == "65299" then
--				gcs:send_text(5,string.format("CAN[%u] msg from " .. id .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
--			end
			return frame
		end
	end
	return false
end

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
max_timeout = 15 -- max connection errors in a row
timeout = 0
start_time = 0
function update()
	-- Go back to idle if lost connection to either thruster
	if not get_frame(1, port_driver) or not get_frame(2, stbd_driver) then
		timeout = timeout+1
		if timeout > max_timeout then
			timeout = max_timeout
			gcs:send_text(0, "Lost connection with TQ")
			set_speed(0, port_driver)
			set_transmission(125, port_driver)
			set_speed(0, stbd_driver)
			set_transmission(125, stbd_driver)
			return check_ready()
		end
	else
		timeout = 0
	end
		
	-- Get direction and motor speed
	if arming:is_armed() then -- only run if armed
		port_speed = SRV_Channels:get_output_scaled(port_ch)
		stbd_speed = SRV_Channels:get_output_scaled(stbd_ch)
	else
		port_speed = 0
		stbd_speed = 0
	end

	port_dir = 125
	stbd_dir = 125
	if port_speed ~= mid_input then
		port_dir = (port_speed > mid_input) and 126 or 124
	end
	if stbd_speed ~= mid_input then
		stbd_dir = (stbd_speed > mid_input) and 126 or 124
	end
	-- Scale throttle to max for TQ
	port_scaled = math.floor(math.abs(((port_speed-mid_input) / (max_input-mid_input)) * max_speed))
	stbd_scaled = math.floor(math.abs(((stbd_speed-mid_input) / (max_input-mid_input)) * max_speed))
		
	-- Set direction and motor speed
	set_transmission(port_dir, port_driver)
	set_transmission(stbd_dir, stbd_driver)
	set_speed(port_scaled, port_driver)
	set_speed(stbd_scaled, stbd_driver)

	return update, 50
end



port_ready = 0
stbd_ready = 0
function check_ready()
	frame = get_frame(1, port_driver)
	if frame then
		pgn = tostring(get_pgn(frame:id()))
		if pgn == TQ_STATUS_PGN then -- check if Torqeedo is ready by this message
			port_ready = frame:data(4) >> 7 -- Check first flag of thruster status bitmap
--			if port_ready == 1 then
--				gcs:send_text(5, "Port thruster ready")
--			end
		end
	end
	
	frame = get_frame(2, stbd_driver)
	if frame then
		pgn = tostring(get_pgn(frame:id()))
		if pgn == TQ_STATUS_PGN then -- check if Torqeedo is ready by this message
			stbd_ready = frame:data(4) >> 7 -- Check first flag of thruster status bitmap
--			if stbd_ready == 1 then
--				gcs:send_text(5, "Starboard thruster ready")
--			end
		end
	end

	-- Both thrusters must be ready simultaneously
	if port_ready == 1 and stbd_ready == 1 then
		gcs:send_text(0, "TQ Ready")
		port_ready = 0 -- reset thruster ready flags
		stbd_ready = 0
		return update, 2000 -- Need to wait for thrusters to really be ready
	else
		return check_ready, 10
	end
end

return check_ready()