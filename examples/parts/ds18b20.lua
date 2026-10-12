-- DS18B20 1-Wire temperature sensor, written against the part scripting API.
-- Terminal: dq (open-drain net with a pull-up).
-- Params: rom ("28-11-22-33-44-55-66", 7 bytes, CRC is appended),
--         temperature_c (25), resolution (12).
-- Exposes: temperature_c.

local RESET_MIN, PRESENCE_DELAY, PRESENCE_LEN = 480 * US, 30 * US, 120 * US
local SAMPLE_DELAY, READ_HOLD = 30 * US, 40 * US

local dq = pins.dq
local temperature = params.temperature_c or 25.0
local resolution = params.resolution or 12
assert(resolution >= 9 and resolution <= 12, "resolution must be 9..12")

local function crc8(bytes, n)
  local crc = 0
  for i = 1, n do
    local b = bytes[i]
    for _ = 1, 8 do
      local mix = (crc ~ b) & 1
      crc = crc >> 1
      if mix == 1 then crc = crc ~ 0x8c end
      b = b >> 1
    end
  end
  return crc
end

local rom = {0x28, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66}
if type(params.rom) == "string" then
  rom = {}
  for hex in params.rom:gmatch("%x+") do rom[#rom + 1] = tonumber(hex, 16) end
  assert(#rom == 7 and rom[1] == 0x28, "rom needs 7 bytes starting with 28")
end
rom[8] = crc8(rom, 7)

local th, tl = 0x4b, 0x46
local config = 0x1f | ((resolution - 9) << 5)
local temp_reg = {0x50, 0x05} -- 85 C until the first conversion

local mode = "idle"
local fall_t, ignore_until, busy_until = 0, 0, 0
local release_id

local function hold_low(t)
  dq:drive("low", t)
  release_id = at(t + READ_HOLD, function(now)
    release_id = nil
    dq:drive("hiz", now)
  end)
end

-- RX state
local rx_left, rx_bytes, rx_bit, rx_shift, rx_done
local function begin_rx(count, done)
  mode, rx_left, rx_bytes, rx_bit, rx_shift, rx_done = "rx", count, {}, 0, 0, done
end
local function rx_bit_in(bit)
  rx_shift = rx_shift | (bit and 1 or 0) << rx_bit
  rx_bit = rx_bit + 1
  if rx_bit < 8 then return end
  rx_bytes[#rx_bytes + 1] = rx_shift
  rx_shift, rx_bit = 0, 0
  rx_left = rx_left - 1
  if rx_left == 0 then
    mode = "idle"
    rx_done()
  end
end

-- TX state
local tx_bytes, tx_index, tx_bit, tx_done
local function begin_tx(bytes, done)
  mode, tx_bytes, tx_index, tx_bit, tx_done = "tx", bytes, 1, 0, done
end
local function tx_slot(t)
  if (tx_bytes[tx_index] >> tx_bit) & 1 == 0 then hold_low(t) end
  tx_bit = tx_bit + 1
  if tx_bit == 8 then
    tx_bit = 0
    tx_index = tx_index + 1
    if tx_index > #tx_bytes then
      mode = "idle"
      if tx_done then tx_done() end
    end
  end
end

local function scratchpad()
  local d = {temp_reg[1], temp_reg[2], th, tl, config, 0xff, 0x0c, 0x10}
  d[9] = crc8(d, 8)
  return d
end

local function alarm_active()
  local raw = temp_reg[1] | temp_reg[2] << 8
  if raw >= 0x8000 then raw = raw - 0x10000 end
  local whole = raw >> 4
  local function s8(v) return v >= 128 and v - 256 or v end
  return whole > s8(th) or whole <= s8(tl)
end

local function conversion_ns()
  return ({[0] = 93750000, 187500000, 375000000, 750000000})[(config >> 5) & 3]
end

local function function_command(cmd)
  if cmd == 0x44 then
    local t = math.max(-55.0, math.min(125.0, temperature))
    local raw = math.floor(t * 16 + 0.5)
    local bits = ((config >> 5) & 3) + 9
    raw = raw & ~((1 << (12 - bits)) - 1)
    raw = raw & 0xffff
    temp_reg[1], temp_reg[2] = raw & 0xff, (raw >> 8) & 0xff
    busy_until = now() + conversion_ns()
    mode = "conv"
  elseif cmd == 0xbe then
    begin_tx(scratchpad(), nil)
  elseif cmd == 0x4e then
    begin_rx(3, function()
      th, tl = rx_bytes[1], rx_bytes[2]
      config = (rx_bytes[3] & 0x60) | 0x1f
      begin_rx(1, function() function_command(rx_bytes[1]) end)
    end)
  elseif cmd == 0xb4 then
    begin_tx({0xff}, nil)
  else
    mode = "idle" -- Copy/Recall are accepted; EEPROM is not modeled
  end
end

local search_bit, search_phase
local function rom_command(cmd)
  if cmd == 0x33 then
    begin_tx({table.unpack(rom)}, function()
      begin_rx(1, function() function_command(rx_bytes[1]) end)
    end)
  elseif cmd == 0xcc then
    begin_rx(1, function() function_command(rx_bytes[1]) end)
  elseif cmd == 0x55 then
    begin_rx(8, function()
      for i = 1, 8 do
        if rx_bytes[i] ~= rom[i] then mode = "idle"; return end
      end
      begin_rx(1, function() function_command(rx_bytes[1]) end)
    end)
  elseif cmd == 0xf0 or (cmd == 0xec and alarm_active()) then
    mode, search_bit, search_phase = "search", 0, 0
  else
    mode = "idle"
  end
end

local function search_slot(t)
  local rom_bit = (rom[search_bit // 8 + 1] >> (search_bit % 8)) & 1 == 1
  if search_phase < 2 then
    local value = rom_bit
    if search_phase == 1 then value = not rom_bit end
    if not value then hold_low(t) end
    search_phase = search_phase + 1
    return
  end
  local sample_at = t + SAMPLE_DELAY
  at(sample_at, function()
    if mode ~= "search" then return end
    if dq:level_at(sample_at) ~= rom_bit then mode = "idle"; return end
    search_phase = 0
    search_bit = search_bit + 1
    if search_bit == 64 then
      begin_rx(1, function() function_command(rx_bytes[1]) end)
    end
  end)
end

local function on_fall(t)
  fall_t = t
  if mode == "rx" then
    local sample_at = t + SAMPLE_DELAY
    at(sample_at, function()
      if mode == "rx" then rx_bit_in(dq:level_at(sample_at)) end
    end)
  elseif mode == "tx" then
    tx_slot(t)
  elseif mode == "search" then
    search_slot(t)
  elseif mode == "conv" then
    if t < busy_until then hold_low(t) end -- busy: the master reads 0
  end
end

local function on_rise(t)
  if t - fall_t < RESET_MIN then return end
  if release_id then cancel(release_id); release_id = nil end
  mode = "idle"
  ignore_until = t + PRESENCE_DELAY + PRESENCE_LEN
  at(t + PRESENCE_DELAY, function()
    dq:drive("low", t + PRESENCE_DELAY)
    release_id = at(t + PRESENCE_DELAY + PRESENCE_LEN, function(now)
      release_id = nil
      dq:drive("hiz", now)
      begin_rx(1, function() rom_command(rx_bytes[1]) end)
    end)
  end)
end

-- Edges up to ignore_until belong to the presence window (ours and other
-- slaves'); edges from our own drive are never delivered to on_edge.
function on_edge(_, high, t)
  if t <= ignore_until then return end
  if high then on_rise(t) else on_fall(t) end
end

expose("temperature_c", function() return temperature end,
       function(v) temperature = v end)
