motor = {}

_G.sys = require "sys"

function motor.parse(order, speed)
    speed = tonumber(string.sub(order, 3, 5))
    -- 前进&刹车
    if string.sub(order, 1, 1) == "w" then
        gpio.setup(17, 0)
        gpio.setup(19, 0)
        if string.sub(order, 2, 2) == "a" then
            log.info("W801", "前进并左转", speed)
            pwm.open(0, 50, speed / 2)
            pwm.open(2, 50, speed)
        elseif string.sub(order, 2, 2) == "d" then
            log.info("W801", "前进并右转", speed)
            pwm.open(0, 50, speed)
            pwm.open(2, 50, speed / 2)
            -- 刹车
        elseif string.sub(order, 2, 2) == "s" then
            log.info("W801", "刹车", speed)
            gpio.setup(16, 0)
            gpio.setup(18, 0)
        else
            log.info("W801", "前进", speed)
            pwm.open(0, 50, speed)
            pwm.open(2, 50, speed)
        end

        -- 后退
    elseif string.sub(order, 1, 1) == "s" then
        gpio.setup(16, 0)
        gpio.setup(18, 0)
        if string.sub(order, 2, 2) == "a" then
            log.info("W801", "后退并左转", speed)
            pwm.open(1, 50, speed / 2)
            pwm.open(3, 50, speed)
        elseif string.sub(order, 2, 2) == "d" then
            log.info("W801", "后退并右转", speed)
            pwm.open(1, 50, speed)
            pwm.open(3, 50, speed / 2)
        else
            log.info("W801", "后退", speed)
            pwm.open(1, 50, speed)
            pwm.open(3, 50, speed)
        end

        -- 左转
    elseif string.sub(order, 1, 2) == "a " then
        log.info("W801", "左转", speed)
        pwm.open(1, 50, speed)
        pwm.open(2, 50, speed)
        gpio.setup(16, 0)
        gpio.setup(19, 0)

        -- 右转
    elseif string.sub(order, 1, 2) == "d " then
        log.info("W801", "右转", speed)
        pwm.open(0, 50, speed)
        pwm.open(3, 50, speed)
        gpio.setup(17, 0)
        gpio.setup(18, 0)

    end
end

return motor
