----- Посевный дрон
----- Первая партия
--------------------
----- Переменные
-- Дрон
local TAKEOFF_POSITION = {["x"] = 1.6, ["y"] = 0.5, ["z"] = 1} -- change
local current_position = {["x"] = 1.6, ["y"] = 0.5} -- change
local Z = 2 -- change
local state = "TAKEOFF"

-- Цель
local target = {} -- change
target[1] = {["x"] = 4.5, ["y"] = 3.2} -- Пожар 1
target[2] = {["x"] = 4.5, ["y"] = 3.2} -- Пожар 2
target[3] = {["x"] = 4.5, ["y"] = 3.2} -- Лес
local i = 3 -- change

-- local PIT_STOP_POS = {["x"] = 0.5, ["y"] = 9.5} -- change
-- local PIT_STOP_TIME = 5 -- change

local pre_sleep_time = 13 - 2 -- change

local going_time = 5 -- change
local returnning_timeout = 0 -- change

local magnet = Gpio.new(Gpio.C, 3, Gpio.OUTPUT)
local drop_height = 1.2 -- change
local current_z = 2 -- change

action = {
    ["TAKEOFF"] = function(x)
        ap.push(Ev.MCE_PREFLIGHT)

        sleep(pre_sleep_time + 2)
        magnet:set()
        sleep(3)

        ap.push(Ev.MCE_TAKEOFF)

        state = "TAKE_HEIGHT"
    end,

    ["TAKE_HEIGHT"] = function(x)
        ap.goToLocalPoint(current_position["x"], current_position["y"], Z)

        state = "GO_ABOVE_TARGET"
    end,

    -- ["PIT_STOP"] = function(x)
    --     -- current_position["x"] = PIT_STOP_POS["x"]
    --     -- current_position["y"] = PIT_STOP_POS["y"]

    --     ap.goToLocalPoint(PIT_STOP_POS["x"], PIT_STOP_POS["y"], Z,
    --                       going_time / 2)

    --     state = "GO_ABOVE_TARGET"
    -- end,

    ["GO_ABOVE_TARGET"] = function(x)
        current_position["x"] = target[i]["x"]
        current_position["y"] = target[i]["y"]

        -- sleep(PIT_STOP_TIME)
        ap.goToLocalPoint(current_position["x"], current_position["y"], Z,
                          going_time / 2)

        state = "DOWN_CLOSER"
    end,

    ["DOWN_CLOSER"] = function(x)
        ap.goToLocalPoint(current_position["x"], current_position["y"],
                          current_z)

        if (current_z <= drop_height) then
            state = "DROP_PAYLOAD"
        else
            current_z = current_z - 0.3 -- change
            state = "DOWN_CLOSER"
        end
    end,

    ["DROP_PAYLOAD"] = function(x)
        sleep(1)
        magnet:reset()

        ap.goToLocalPoint(current_position["x"], current_position["y"], Z)

        state = "RETURN_HOME"
    end,

    ["RETURN_HOME"] = function(x)
        sleep(returnning_timeout)
        ap.goToLocalPoint(TAKEOFF_POSITION["x"], TAKEOFF_POSITION["y"], Z)

        state = "PRELANDING"
    end,

    ["PRELANDING"] = function(x)
        ap.goToLocalPoint(TAKEOFF_POSITION["x"], TAKEOFF_POSITION["y"],
                          TAKEOFF_POSITION["z"] + 0.5)
        state = "LANDING"
    end,

    ["LANDING"] = function(x)
        ap.push(Ev.MCE_LANDING)

        state = "DISARM"
    end,

    ["DISARM"] = function(x)
        ap.push(Ev.ENGINES_DISARM)

        state = "NONE"
    end
}

-----------
--- Ignore

function callback(event)
    if (event == Ev.POINT_REACHED) then action[state]() end
    if (event == Ev.TAKEOFF_COMPLETE) then action[state]() end
    if (event == Ev.COPTER_LANDED) then action[state]() end
end

function start()
    -- считывание значения каналов пульта управления
    _, _, _, _, _, _, _, ch8 = Sensors.rc()
    if (ch8 > 0) then
        action[state]()
    else
        Timer.callLater(0.2, start)
    end
end

start()
