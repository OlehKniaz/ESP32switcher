---------------------------------------------------------
-- RF Failover Script (TX12 MK2 Compatible)
-- Auto-switch Internal ELRS 2.4GHz ↔ External ELRS 915MHz
---------------------------------------------------------

local FAIL_THRESHOLD = 20
local RECOVER_THRESHOLD = 40
local FAIL_TIME = 1.0
local RECOVER_TIME = 2.0

local lastFail = 0
local lastRecover = 0
local state = "2.4G"

----------------------------------------------------------------
-- EdgeTX MK2 RF control API
----------------------------------------------------------------
local function setInternalRF(enable)
    model.setModule(0, { type = 3, rfEnable = enable })  
    -- module 0 = internal, type 3 = CRSF (ELRS)
end

local function setExternalRF(enable)
    model.setModule(1, { type = 3, rfEnable = enable })
    -- module 1 = external bay
end

local function activate24()
    state = "2.4G"
    setInternalRF(true)
    setExternalRF(false)
end

local function activate915()
    state = "915M"
    setInternalRF(false)
    setExternalRF(true)
end

----------------------------------------------------------------
-- INIT
----------------------------------------------------------------
local function init()
    activate24()
end

----------------------------------------------------------------
-- MAIN LOOP
----------------------------------------------------------------
local function run()
    lcd.clear()
    local rqly = getValue("RQly") or -1

    lcd.drawText(5, 5, "RF FAILOVER (TX12 MK2)", MIDSIZE)
    lcd.drawText(5, 25, "RQly: " .. rqly, 0)
    lcd.drawText(5, 40, "Active: " .. state, MIDSIZE)

    local now = getTime() / 100

    -- FAIL SWITCH
    if rqly >= 0 and rqly < FAIL_THRESHOLD then
        if lastFail == 0 then lastFail = now end
        if now - lastFail >= FAIL_TIME and state == "2.4G" then
            activate915()
        end
    else
        lastFail = 0
    end

    -- RECOVER SWITCH
    if rqly > RECOVER_THRESHOLD then
        if lastRecover == 0 then lastRecover = now end
        if now - lastRecover >= RECOVER_TIME and state == "915M" then
            activate24()
        end
    else
        lastRecover = 0
    end

    return 0
end

return { init = init, run = run }
