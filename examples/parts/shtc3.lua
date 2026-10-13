-- Sensirion SHTC3 temperature/humidity sensor on I2C (address 0x70), as used
-- on the LYWSD03MMC hardware 1.4.
--
-- Commands (16 bit, MSB first): 0x3517 wake-up, 0xB098 sleep, 0x805D soft
-- reset, 0x7CA2 measure with clock stretching (temperature first). The
-- measurement reply is T msb, T lsb, CRC, RH msb, RH lsb, CRC. Clock stretching
-- is not modeled: the data is simply ready for the read. A measurement while
-- asleep is NACKed, as on the chip.
-- Terminals: sda, scl. Params: address (0x70), temperature_c (25), humidity (50).
-- Exposes: temperature_c, humidity.

local address = params.address or 0x70
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

local asleep = true
local command, command_bytes = 0, 0
local reply, reply_index = {}, 0

local function measure()
  -- T = -45 + 175 * raw / 2^16, RH = 100 * raw / 2^16 (datasheet).
  local t = math.floor((temperature + 45) / 175 * 65536 + 0.5)
  local h = math.floor(humidity / 100 * 65536 + 0.5)
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
      if command == 0x3517 then asleep = false
      elseif command == 0xB098 then asleep = true
      elseif command == 0x805D then asleep = true
      elseif command == 0x7CA2 and not asleep then measure()
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
