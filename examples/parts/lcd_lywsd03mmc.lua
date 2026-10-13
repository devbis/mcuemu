-- The segment LCD of the LYWSD03MMC hardware 1.4: a 6-byte display RAM behind
-- an I2C controller at 7-bit address 0x3C (8-bit 0x78). It decodes what the
-- firmware shows, so a scenario or a test can check the display.
--
-- Protocol, from the Zigbee firmware (lcd.c) and not from a datasheet: every
-- byte is preceded by a control byte. 0x80 starts a command byte (0x40 sets the
-- RAM write address to 0 -- inferred; the init sequence 0x3B 0x02 0x0F ... is
-- accepted and ignored), 0xC0 starts a data byte that goes to the RAM and
-- advances the pointer. Segment layout (see lcd.c): bytes 5,4,3 hold the three
-- big digits (tenths of a degree), byte 5 bit 3 = a leading "1", byte 5 bit 1 =
-- minus, byte 4 bit 3 = decimal point; bytes 1,0 hold the two small digits
-- (humidity), byte 0 bit 3 = percent sign; byte 2 bits 7..5 select the unit
-- (0xA0 = C, 0x60 = F), bit 4 = BLE, bits 2..0 = smiley; byte 1 bit 3 = battery.
-- Terminals: sda, scl. Params: address (0x3C).
-- Exposes (read-only): temperature (tenths of a degree, -9999 when blank),
--   humidity (%, -1 when blank), unit (0 none, 1 C, 2 F) and seg0..seg5 (the raw
--   display RAM). The decoded line is also logged whenever it changes.

local address = params.address or 0x3C

local digits = {
  [0xF5] = 0, [0x05] = 1, [0xD3] = 2, [0x97] = 3, [0x27] = 4,
  [0xB6] = 5, [0xF6] = 6, [0x15] = 7, [0xF7] = 8, [0xB7] = 9,
}
local ram = {0, 0, 0, 0, 0, 0}
local mode, pointer = "control", 0
local control_is_data = false
local last_text = nil

local function digit(byte)
  return digits[byte & 0xF7]
end

local function temperature()
  local d3, d4, d5 = digit(ram[4]), digit(ram[5]), digit(ram[6])
  if not d3 then return nil end
  local value = d3 + (d4 or 0) * 10 + (d5 or 0) * 100
  if ram[6] & 0x08 ~= 0 then value = value + 1000 end
  -- Bit 1 of byte 5 is the minus sign only while the hundreds digit is blank:
  -- otherwise it is part of the digit's pattern (the firmware cannot show both,
  -- so values below -9.9 are not representable).
  if not d5 and ram[6] & 0x02 ~= 0 then value = -value end
  return value
end

local function humidity()
  local d0, d1 = digit(ram[1]), digit(ram[2])
  if not d0 then return nil end
  return d0 + (d1 or 0) * 10
end

local function unit()
  local u = ram[3] & 0xE0
  if u == 0xA0 then return 1 elseif u == 0x60 then return 2 end
  return 0
end

local function describe()
  local t, h = temperature(), humidity()
  local text = "blank"
  if t or h then
    text = string.format("%s%s  %s", t and string.format("%.1f", t / 10) or "--",
      ({[0] = "", "C", "F"})[unit()], h and (h .. "%") or "--")
  end
  if text ~= last_text then
    last_text = text
    log("display: " .. text)
  end
end

i2c_slave {
  address = address,
  on_start = function() mode, control_is_data = "control", false end,
  on_stop = function() describe() end,
  on_write = function(byte)
    if mode == "control" then
      control_is_data = byte & 0x40 ~= 0
      mode = "byte"
    else
      if control_is_data then
        if pointer < 6 then ram[pointer + 1] = byte end
        pointer = pointer + 1
      elseif byte & 0xC0 == 0x40 then
        pointer = byte & 0x3F
      end
      mode = "control"
    end
    return true
  end,
}

expose("temperature", function() return temperature() or -9999 end)
expose("humidity", function() return humidity() or -1 end)
expose("unit", unit)
for i = 1, 6 do
  expose("seg" .. (i - 1), function() return ram[i] end)
end
