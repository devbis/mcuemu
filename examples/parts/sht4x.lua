-- Sensirion SHT4x temperature/humidity sensor on I2C (address 0x44), as used
-- on the LYWSD03MMC hardware 1.5 and later.
--
-- A one-byte command starts a measurement (0xFD high precision, 0xF6 medium,
-- 0xE0 low); 0x94 is a soft reset. The reply is T msb, T lsb, CRC, RH msb,
-- RH lsb, CRC. The conversion time is not modeled: the data is ready at once.
-- Terminals: sda, scl. Params: address (0x44), temperature_c (25), humidity (50).
-- Exposes: temperature_c, humidity.

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

local reply, reply_index = {}, 0

local function measure()
  -- T = -45 + 175 * raw / 65535, RH = -6 + 125 * raw / 65535 (datasheet).
  local t = math.floor((temperature + 45) / 175 * 65535 + 0.5)
  local h = math.floor((humidity + 6) / 125 * 65535 + 0.5)
  t = math.max(0, math.min(65535, t))
  h = math.max(0, math.min(65535, h))
  local tm, tl, hm, hl = t >> 8, t & 0xff, h >> 8, h & 0xff
  reply = {tm, tl, crc8(tm, tl), hm, hl, crc8(hm, hl)}
  reply_index = 0
end

i2c_slave {
  address = address,
  on_write = function(byte)
    if byte == 0xFD or byte == 0xF6 or byte == 0xE0 then measure()
    elseif byte ~= 0x94 then return false end
    return true
  end,
  on_read = function()
    reply_index = reply_index + 1
    return reply[reply_index] or 0xff
  end,
}

expose("temperature_c", function() return temperature end,
       function(v) temperature = v end)
expose("humidity", function() return humidity end, function(v) humidity = v end)
