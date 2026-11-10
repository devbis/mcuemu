-- TI OPT3001 ambient light sensor (I2C, 0x44..0x47): 16-bit big-endian registers behind a pointer byte. Registers: 0x00 result
-- (exponent in bits 15:12, mantissa in 11:0, lux = 0.01 * 2^E * M), 0x01 configuration (default 0xC810; bit 7 is the conversion-ready
-- flag, always reported set, as the conversion takes no simulated time), 0x02 low limit, 0x03 high limit, 0x7E manufacturer id
-- 0x5449, 0x7F device id 0x3001. Seen on the Tuya ZS3L ZSS-ZK-THL thermometer next to a CHT8305: it reads 0x7E, writes the limits
-- and the configuration 0xCE11, then reads the result.
-- Terminals: sda, scl. Params: address (0x44), lux (100). Exposes: lux, `reads`, `writes`.

local address = params.address or 0x44
local lux = params.lux or 100.0

local registers = {[0x01] = 0xC810, [0x02] = 0xC0, [0x03] = 0, [0x7e] = 0x5449, [0x7f] = 0x3001}
local pointer, bytes_in, latched = 0, 0, 0
local read_index, reads, writes = 0, 0, 0

local function result()
  local exponent = 0
  while exponent < 11 and lux / (0.01 * 2 ^ exponent) > 4095 do exponent = exponent + 1 end
  local mantissa = math.max(0, math.min(4095, math.floor(lux / (0.01 * 2 ^ exponent) + 0.5)))
  return (exponent << 12) | mantissa
end

local function read_register(reg)
  if reg == 0x00 then return result() end
  local value = registers[reg] or 0
  if reg == 0x01 then value = value | 0x0080 end
  return value
end

i2c_slave {
  address = address,
  on_start = function() bytes_in = 0 end,
  on_write = function(byte)
    bytes_in = bytes_in + 1
    if bytes_in == 1 then pointer = byte; read_index = 0
    elseif bytes_in == 2 then latched = byte << 8
    elseif bytes_in == 3 then registers[pointer] = latched | byte; writes = writes + 1
    else return false end
    return true
  end,
  on_read = function()
    local value = read_register(pointer)
    local byte = (read_index == 0) and (value >> 8) or (value & 0xff)
    read_index = read_index + 1
    if read_index == 2 then read_index = 0; reads = reads + 1 end
    return byte
  end,
}

expose("lux", function() return lux end, function(v) lux = v end)
expose("reads", function() return reads end)
expose("writes", function() return writes end)
