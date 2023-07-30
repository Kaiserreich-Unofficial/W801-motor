Blink = {}

local sys = require'sys'

function Blink.parse(order)
    local LED1 = gpio.setup(41, 1)
    local LED2 = gpio.setup(42, 1)
    local LED3 = gpio.setup(34, 1)
    local LED4 = gpio.setup(33, 1)

    --前进&刹车
    if string.sub(order,1,1) == "w" then
        LED1(0)
        if string.sub(order,2,2) == "a" then
            LED3(0)
        elseif string.sub(order,2,2) == "d" then
            LED4(0)
        elseif string.sub(order,2,2) == "s" then
            LED1(1)
            LED2(1)
            LED3(1)
            LED4(1)
        end

    --后退
    elseif string.sub(order,1,1) == "s" then
        LED2(0)
        if string.sub(order,2,2) == "a" then
            LED3(0)
        elseif string.sub(order,2,2) == "d" then
            LED4(0)
        end

    --左转
    elseif string.sub(order,1,2) == "a " then
        LED3(0)
    --右转
    elseif string.sub(order,1,2) == "d " then
        LED4(0)
    end

end

return Blink
