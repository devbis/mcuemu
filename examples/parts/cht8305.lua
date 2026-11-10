-- Sensylink CHT8305 I2C temperature/humidity sensor (Tuya TS0222 and others): a register pointer is written, then 16-bit
-- big-endian values are read; the pointer advances over reads. Registers: 0x00 temperature, 0x01 humidity, 0x02 configuration
-- (writable, default 0x1004), 0xFE manufacturer id 0x5959, 0xFF device id 0x8305. Temperature is
-- raw / 65536 * 165 - 40 degC and humidity raw / 65536 * 100 %RH, in the upper 14 bits (the low two read 0), as in the
-- datasheet. The measurement is taken when the pointer is set, so a read right after sees the current values.
-- Terminals: sda, scl. Params: address (0x40), temperature_c (25), humidity (50). Exposes: temperature_c, humidity.

local address = params.address or 0x40
local temperature = params.temperature_c or 25.0
local humidity = params.humidity or 50.0

local registers = {[0x02] = 0x1004, [0xfe] = 0x5959, [0xff] = 0x8305}
local pointer, written, bytes_in, latched = 0, false, 0, 0

local function raw(value, scale, offset)
  local v = math.floor((value + offset) / scale * 65536 + 0.5)
  return (math.max(0, math.min(65535, v))) & 0xfffc
end

local function read_register(reg)
  if reg == 0x00 then return raw(temperature, 165, 40)
  elseif reg == 0x01 then return raw(humidity, 100, 0) end
  return registers[reg] or 0
end

local read_value, read_index = 0, 0

i2c_slave {
  address = address,
  on_start = function() written, bytes_in = false, 0 end,
  on_write = function(byte)
    bytes_in = bytes_in + 1
    if bytes_in == 1 then
      pointer = byte
      read_value, read_index = read_register(pointer), 0
    elseif bytes_in == 2 then latched = byte << 8
    elseif bytes_in == 3 then
      if pointer == 0x02 then registers[0x02] = latched | byte end
    else return false end
    return true
  end,
  on_read = function()
    if read_index == 0 then read_value = read_register(pointer) end
    local byte = (read_index == 0) and (read_value >> 8) or (read_value & 0xff)
    read_index = read_index + 1
    if read_index == 2 then read_index = 0 end
    return byte
  end,
}

expose("temperature_c", function() return temperature end, function(v) temperature = v end)
expose("humidity", function() return humidity end, function(v) humidity = v end)
