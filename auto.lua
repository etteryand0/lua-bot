----------------------------
-- Оранжевый дрон сверху
local state = "TAKEOFF"
local Z = 1.5 -- требуемая высота
local TAKEOFF_POSITION = {["x"] = 0.5, ["y"] = 5.5, ["z"] = 1} -- начальная позиция дрона
local current_position = TAKEOFF_POSITION

-- Поле
local ZONE_SIZE = {["width"] = 6, ["height"] = 6} -- ширина и длина в метрах
local ZONE_SQUARES_COUNT = {["x"] = 6, ["y"] = 6} -- ширина и длина в кадрах
local MAX_LOOPS = ZONE_SQUARES_COUNT["y"] / 2
local SQUARE_SIZE = {
    ["width"] = ZONE_SIZE["width"] / ZONE_SQUARES_COUNT["x"],
    ["height"] = ZONE_SIZE["height"] / ZONE_SQUARES_COUNT["y"]
} -- Размеры одного квадрата
local BORDER_POS = {
    ["left"] = SQUARE_SIZE["width"] / 2,
    ["right"] = ZONE_SIZE["width"] - SQUARE_SIZE["width"] / 2
} -- Границы, за которые нельзя вылетать. Край в общем

-- Фото
local PHOTO_STEP = 1 -- через каждый шаг (м) делать фото
local VELOCITY = 0.1 -- скорость передвижения между шагами
local FLIGHT_TIME = PHOTO_STEP / VELOCITY
local loop_id = 1 -- номер цикла


action = {
    ["TAKEOFF"] = function(x)
        ap.push(Ev.MCE_PREFLIGHT)
        sleep(3)
        ap.push(Ev.MCE_TAKEOFF)

        state = "GO_TO_START"
    end,

    ["GO_TO_START"] = function(x) -- перейти в позицию для начала цикла
        current_position["y"] = TAKEOFF_POSITION["y"] - SQUARE_SIZE["height"]

        timer_uart:start()
        ap.goToLocalPoint(current_position["x"], current_position["y"], Z, 5)

        state = "GO_FORWARD"
    end,

    ["GO_FORWARD"] = function(x)
        is_moved = false

        -- CAMERA-SHOOT
        if (loop_id % 2 ~= 0) then -- Номер цикла нечётный. Направо
            if (go_further(1) == true) then
                is_moved = true
                current_position["x"] = current_position["x"] + PHOTO_STEP
            else
                current_position["x"] = BORDER_POS["right"]
            end
        else -- Номер цикла чётный. Налево
            if (go_further(0) == true) then
                is_moved = true
                current_position["x"] = current_position["x"] - PHOTO_STEP
            else
                current_position["x"] = BORDER_POS["left"]
            end
        end

        ap.goToLocalPoint(current_position["x"], current_position["y"], Z, FLIGHT_TIME)
        if (is_moved == true) then -- Шагнул
            state = "GO_FORWARD"
        else -- Не шагнул, нужно сменить траекторию
            state = "NEXT_LOOP"
        end
    end,

    ["NEXT_LOOP"] = function(x)
        if (loop_id == MAX_LOOPS) then
            timer_uart:stop()
            ap.goToLocalPoint(TAKEOFF_POSITION["x"], TAKEOFF_POSITION["y"], Z, 20)

            state = "FINISH"
            return true
        end
        current_position["y"] = current_position["y"] - SQUARE_SIZE["height"]
        ap.goToLocalPoint(current_position["x"], current_position["y"], Z, FLIGHT_TIME)

        loop_id = loop_id + 1
        state = "GO_FORWARD"
    end,

    ["FINISH"] = function(x)
        ap.push(Ev.MCE_LANDING)

        state = "DISARM"
    end,

    ["DISARM"] = function(x)
        ap.push(Ev.ENGINES_DISARM)
        state = "NONE"
    end
}

function go_further(direction) -- Возвращает boolean. Идти дальше или нет
    if (direction == 1) then -- Направо
        -- Не выходит за рамки - true
        return BORDER_POS["right"] > (current_position["x"] + PHOTO_STEP)
    end

    -- Налево
    return BORDER_POS["left"] < (current_position["x"] - PHOTO_STEP)
end

function callback(event)
    if (event == Ev.POINT_REACHED) then
        sleep(1)
        action[state]()
    end

    if (event == Ev.TAKEOFF_COMPLETE) then action[state]() end
end
