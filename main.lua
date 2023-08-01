-- LuaTools需要PROJECT和VERSION这两个信息
PROJECT = "W801-MOTOR"
VERSION = "1.0.0"

-- 引入必要的库文件(lua编写), 内部库不需要require
_G.sys = require "sys"
_G.udpsrv = require "udpsrv"
_G.motor = require "W801-CAR"
_G.Blink = require "Blink"
_G.madgwick = require "madgwick"
require "hmc5883"
require "mpu6xxx"

imu_mag = madgwick:new()
local deg2rad = math.pi / 180 -- 角度转弧度的系数

-- 初始化GPS串口
uart.setup(1, 9600)
uart.on(1, "recv", function(id, len)
    local data = uart.read(1, 1024)
    libgnss.parse(data)
    sys.publish("GPS_ONLINE")
end)

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

-- 加速度计读数
sys.taskInit(function()
    init_mpu6050() -- 有wait不能放在外面
    while 1 do
        -- 初始化hmc588配置
        hmc5883l_init()

        local accel = mpu6xxx_get_accel()
        local gyro = mpu6xxx_get_gyro()
        local roll = math.atan2(accel.y, accel.z) / deg2rad
        local pitch = math.atan2((-accel.x), math.sqrt(accel.y ^ 2 + accel.z ^ 2)) / deg2rad

        -- 读取x,y,z数值
        local yaw, mag_x, mag_y, mag_z = hmc5883l_read()

        -- madgwick梯度下降法解算姿态
        local roll1, pitch1, yaw1 = imu_mag:updateIMU(accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, mag_x, mag_y,
            mag_z)
        log.info("Accel", "x", accel.x, "y", accel.y, "z", accel.z)
        log.info("GYRO", "x", gyro.x, "y", gyro.y, "z", gyro.z)
        log.info("Raw_Euler", "Roll", roll, "Pitch", pitch, "Yaw", yaw)
        log.info("Timestamp",mcu.ticks())
        log.info("madgwick_Euler", "Roll", roll1, "Pitch", pitch1, "Yaw", yaw1)
        sys.wait(100)
    end
end)

-- GPS读数
sys.taskInit(function()
    sys.waitUntil("GPS_ONLINE")
    while 1 do
        local rmc = libgnss.getRmc()
        local lat, long, speed
        if json.encode(rmc) then
            lat, long, speed = rmc.lat * 0.01, rmc.lng * 0.01, rmc.speed * 1852 / 3600
            log.info("Location", lat .. "N", long .. "E")
            log.info("Speed", speed)
        end
        sys.wait(1000)
    end
end)

-- 用户代码已结束---------------------------------------------
-- 结尾总是这一句
sys.run()
-- sys.run()之后后面不要加任何语句!!!!!
