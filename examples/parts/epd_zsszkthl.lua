-- The e-ink display of the Moes ZSS-ZK-THL thermometer (YMCE1031-0252AAAMFGN), as far as the Tuya ZS3L firmware shows it: a
-- 3-wire SPI with a 9-bit frame, the data/command bit first and then eight data bits MSB first, sampled on the rising edge of
-- SCK while CS is low (one frame per CS pulse). Command set seen from the firmware (names are guesses): 0x04, 0x00, 0x01, 0x30,
-- 0x03 initialisation; 0x20, 0x23, 0x24, 0x25, 0x26 each followed by a 15-byte waveform table; 0x18 followed by 18 bytes of
-- segment RAM; 0x11 0xFF starts the refresh.
-- On the board (found from the firmware's GPIO traffic): SCK = PD0, DATA = PC2, CS = PB1.
-- Terminals: sck, data, cs. Exposes (read-only): `ram` (the 18 segment bytes of the last refresh, hex), `refreshes`,
-- `frames` (completed 9-bit frames) and `commands` (the command bytes seen since the last refresh, hex). The segment layout is not
-- decoded: only the temperature moves the RAM in the first seconds, in a bit pattern that is not plain 7-segment bytes.

local bits, in_frame = {}, false
local level = {sck = false, data = false, cs = true}
local command, pending, ram, last_ram = nil, {}, {}, {}
local frames, refreshes, commands = 0, 0, {}

local function hex(list)
  local out = {}
  for i, b in ipairs(list) do out[i] = string.format("%02x", b) end
  return table.concat(out)
end

local function finish_frame()
  if #bits ~= 9 then return end
  local value = 0
  for i = 2, 9 do value = (value << 1) | bits[i] end
  frames = frames + 1
  if bits[1] == 0 then
    command = value
    commands[#commands + 1] = string.format("%02x", value)
    if command == 0x18 then ram = {} end
  elseif command == 0x18 then
    ram[#ram + 1] = value
  elseif command == 0x11 then
    refreshes = refreshes + 1
    last_ram = ram
    commands = {}
  end
end

function on_edge(terminal, high, t)
  level[terminal] = high
  if terminal == "cs" then
    if not high then bits, in_frame = {}, true
    elseif in_frame then in_frame = false; finish_frame() end
  elseif terminal == "sck" and high and in_frame then
    bits[#bits + 1] = level.data and 1 or 0
  end
end

expose("ram", function() return hex(last_ram) end)
expose("refreshes", function() return refreshes end)
expose("frames", function() return frames end)
expose("commands", function() return table.concat(commands, " ") end)
