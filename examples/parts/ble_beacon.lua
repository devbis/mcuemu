-- A scannable BLE advertiser as a script part: ADV_IND with a complete local
-- name on channels 37, 38 and 39 (1 ms apart) every `interval_ms` (default 100),
-- and a SCAN_RSP with manufacturer data to whoever sends a SCAN_REQ. It does not
-- accept connections (the link layer of a peripheral is not scripted here).
--
-- The part needs `radio: { air: ..., protocols: [ble] }`. Frames carry whole
-- link-layer PDUs (`ll = true`), which is what the virtual BLE host expects.
-- Its address is C6:55:44:33:22:11 (static random).
local addr = string.char(0x11, 0x22, 0x33, 0x44, 0x55, 0xC6) -- on-air order
local name = params.name or "mcuemu"
local interval = (tonumber(params.interval_ms) or 100) * MS

local ad = string.char(0x02, 0x01, 0x06, #name + 1, 0x09) .. name
local adv = string.char(0x40, 6 + #ad) .. addr .. ad           -- ADV_IND, TxAdd = random
local rsp_ad = string.char(0x05, 0xFF, 0x59, 0x00, 0x01, 0x02)
local rsp = string.char(0x44, 6 + #rsp_ad) .. addr .. rsp_ad   -- SCAN_RSP, TxAdd = random

local function send(channel)
  radio.listen("ble", channel) -- for a SCAN_REQ after this advertisement
  radio.transmit{protocol = "ble", channel = channel, payload = adv, ll = true}
end

local function event()
  for i = 0, 2 do
    after(i * 1 * MS, function() send(37 + i) end)
  end
  after(interval, event)
end
after(1 * MS, event)

function on_frame(f)
  -- SCAN_REQ (type 3): ScanA, then our address as AdvA.
  if f.ll and f.access_address == 0x8E89BED6 and #f.payload == 14
     and (f.payload:byte(1) & 0x0F) == 3 and f.payload:sub(9, 14) == addr then
    radio.transmit{protocol = "ble", channel = f.channel, payload = rsp, ll = true,
                   t = f.end_ns + 150 * US}
  end
end
