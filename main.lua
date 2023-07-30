-- LuaTools需要PROJECT和VERSION这两个信息
PROJECT = "WLAN-TEST"
VERSION = "1.0.0"

-- 引入必要的库文件(lua编写), 内部库不需要require
_G.sys = require "sys"
_G.udpsrv = require "udpsrv"
_G.motor = require "W801-CAR"
_G.Blink = require "Blink"

sys.taskInit(function()
    local device_id = mcu.unique_id():toHex()
    -- wifi 联网, ESP32系列均支持
    local ssid = "Dragonborn"
    local password = "zby2001528"
    log.info("wifi", ssid)
    wlan.init()
    wlan.setMode(wlan.STATION) -- 默认也是这个模式,不调用也可以
    device_id = wlan.getMac():toHex()
    wlan.connect(ssid, password, 1)
end)

sys.subscribe("IP_READY", function()
    local LED = gpio.setup(27, 0)
    log.info("sntp", "Wifi联网成功,开始NTP时间同步!")
    socket.sntp()
    sys.publish("NTP_OK")
end)

sys.taskInit(function()
    sys.waitUntil("NTP_OK")
    local mytopic = "Controller"
    local srv = udpsrv.create(11451, mytopic)
    sys.publish("UDP_SRV_OK")
    if srv then
        log.info("udpsrv", "启动成功")
        local LED = gpio.setup(32, 0)
        -- 广播
        srv:send("Online", "255.255.255.255", 777)
        while 1 do
            local ret, data = sys.waitUntil(mytopic, 15000)
            if ret then
                -- 按业务处理收到的数据
                motor.parse(data)
                Blink.parse(data)
            else
                log.info("udpsrv", "No Data Received, Keep Alive.")
                srv:send("Online", "255.255.255.255", 777)
            end
        end
    else
        log.info("udpsrv", "启动失败")
        local LED = gpio.setup(32, 1)
    end
end)

sys.subscribe("NTP_UPDATE", function()
    log.info("sntp", "time", os.date())
end)
sys.subscribe("NTP_ERROR", function()
    log.info("socket", "sntp error")
    socket.sntp()
end)

-- 用户代码已结束---------------------------------------------
-- 结尾总是这一句
sys.run()
-- sys.run()之后后面不要加任何语句!!!!!
