-- BL0937 energy-metering IC, the kind found in Tuya smart plugs. It has no
-- digital interface: it reports through two frequency outputs and the MCU
-- counts the pulses.
--
--   CF   one pulse per unit of energy; the frequency follows the active power.
--   CF1  a square wave; SEL picks what it carries: SEL low = current RMS,
--        SEL high = voltage RMS. The MCU flips SEL and times CF1.
--
-- Terminals: cf, cf1 (outputs, idle high), sel (input, optional: without it
--            CF1 carries the current).
-- Params (all optional):
--   voltage_v (230), current_a (0), power_factor (1)   the load
--   power_w                  the load as active power; overrides current_a
--   cf_hz_per_w (0.5)      calibration: CF Hz per watt
--   cf1_hz_per_v (8)       calibration: CF1 Hz per volt
--   cf1_hz_per_a (100)     calibration: CF1 Hz per ampere
--   cf_pulse_us (38)       width of a CF pulse
--   max_hz (10000)         frequencies are clamped to this
-- Exposes (settable): voltage_v, current_a, power_factor, power_w (setting it
--   adjusts the current), and read-only cf_hz and cf1_hz.
--
-- The calibration defaults are round numbers, not the chip's: they come from
-- the shunt and divider on the board, so copy the ratios your firmware uses
-- to turn pulses into watts. A polarity of "pulse low" for CF is modeled; the
-- firmware counts falling edges.

local cf, cf1, sel = pins.cf, pins.cf1, pins.sel
assert(cf and cf1, "bl0937 needs the cf and cf1 terminals")

local voltage = params.voltage_v or 230.0
local current = params.current_a or 0.0
local power_factor = params.power_factor or 1.0
if params.power_w then
  current = params.power_w / (voltage * power_factor)
end
local cf_hz_per_w = params.cf_hz_per_w or 0.5
local cf1_hz_per_v = params.cf1_hz_per_v or 8.0
local cf1_hz_per_a = params.cf1_hz_per_a or 100.0
local pulse_ns = math.floor((params.cf_pulse_us or 38) * US)
local max_hz = params.max_hz or 10000.0
local MIN_HZ = 0.001 -- below this the output stays idle

local function clamp(f)
  if f ~= f or f < MIN_HZ then return 0.0 end
  return math.min(f, max_hz)
end

local function power() return voltage * current * power_factor end
local function cf_hz() return clamp(power() * cf_hz_per_w) end

local function selects_voltage()
  return sel ~= nil and sel:level() == true
end
local function cf1_hz()
  if selects_voltage() then return clamp(voltage * cf1_hz_per_v) end
  return clamp(current * cf1_hz_per_a)
end

local cf_timer, cf1_timer

local function restart_cf()
  if cf_timer then cancel(cf_timer) end
  cf_timer = nil
  cf:drive("high")
  local f = cf_hz()
  if f == 0 then return end
  local period = math.floor(S / f)
  local width = math.min(pulse_ns, period // 2)
  local function pulse(t)
    cf:drive("low", t)
    at(t + width, function(rise) cf:drive("high", rise) end)
    cf_timer = at(t + period, pulse)
  end
  cf_timer = after(period, pulse)
end

local function restart_cf1()
  if cf1_timer then cancel(cf1_timer) end
  cf1_timer = nil
  cf1:drive("low")
  local f = cf1_hz()
  if f == 0 then return end
  local half = math.max(1, math.floor(S / f) // 2)
  local high = false
  local function toggle(t)
    high = not high
    cf1:drive(high and "high" or "low", t)
    cf1_timer = at(t + half, toggle)
  end
  cf1_timer = after(half, toggle)
end

local function restart()
  restart_cf()
  restart_cf1()
end

function on_edge(terminal)
  if terminal == "sel" then restart_cf1() end
end

expose("voltage_v", function() return voltage end,
       function(v) voltage = tonumber(v); restart() end)
expose("current_a", function() return current end,
       function(v) current = tonumber(v); restart() end)
expose("power_factor", function() return power_factor end,
       function(v) power_factor = tonumber(v); restart() end)
expose("power_w", power, function(w)
  local denominator = voltage * power_factor
  current = denominator > 0 and tonumber(w) / denominator or 0.0
  restart()
end)
expose("cf_hz", cf_hz)
expose("cf1_hz", cf1_hz)

restart()
