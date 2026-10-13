-- SHT3x-style I2C temperature/humidity sensor (single-shot, no clock
-- stretching): the master writes a 16-bit command, then reads 6 bytes. Also
-- answers soft reset (0x30A2), clear status (0x3041) and read status (0xF32D,
-- 3 bytes), which drivers send during start-up.
-- Terminals: sda, scl. Params: address (0x44), temperature_c (25),
-- humidity (50). Exposes: temperature_c, humidity.

local address = params.address or 0x44
local temperature = params.temperature_c or 25.0
local humidity = params.humidity or 50.0

local function crc8(msb, lsb)
  local crc = 0xff
  for _, byte in ipairs({msb, lsb}) do
    crc = crc ~ byte
    for _ = 1, 8 do
      if crc & 0x80 ~= 0 then crc = ((crc << 1) ~ 0x31) & 0xff
      else crc = (crc << 1) & 0xff end
    end
  end
  return crc
end

local command, command_bytes = 0, 0
local reply, reply_index = {}, 0

local function status()
  -- 0x8010 is the datasheet's value right after reset (alert pending, reset
  -- detected).
  reply = {0x80, 0x10, crc8(0x80, 0x10)}
  reply_index = 0
end

local function measure()
  local t = math.floor((temperature + 45) / 175 * 65535 + 0.5)
  local h = math.floor(humidity / 100 * 65535 + 0.5)
  t = math.max(0, math.min(65535, t))
  h = math.max(0, math.min(65535, h))
  local tm, tl, hm, hl = t >> 8, t & 0xff, h >> 8, h & 0xff
  reply = {tm, tl, crc8(tm, tl), hm, hl, crc8(hm, hl)}
  reply_index = 0
end

i2c_slave {
  address = address,
  on_start = function() command, command_bytes = 0, 0 end,
  on_write = function(byte)
    command = (command << 8) | byte
    command_bytes = command_bytes + 1
    if command_bytes == 2 then
      if command == 0x2400 or command == 0x2c06 then measure()
      elseif command == 0xf32d then status()
      elseif command == 0x30a2 or command == 0x3041 then reply, reply_index = {}, 0
      else return false end
    end
    return command_bytes <= 2
  end,
  on_read = function()
    reply_index = reply_index + 1
    return reply[reply_index] or 0xff
  end,
}

expose("temperature_c", function() return temperature end,
       function(v) temperature = v end)
expose("humidity", function() return humidity end, function(v) humidity = v end)
