_G.sys = require "sys"

-- pm.wake("mpu6xxx")

i2cid = 0 -- i2cid

i2cslaveaddr = MPU6XXX_ADDRESS_AD0_LOW
deviceid = MPU6050_WHO_AM_I

MPU6XXX_ADDRESS_AD0_LOW = 0x68 -- address pin low (GND), default for InvenSense evaluation board
MPU6XXX_ADDRESS_AD0_HIGH = 0x69 -- address pin high (VCC)

---器件通讯地址
MPU6050_WHO_AM_I = 0x68 -- mpu6050
MPU6500_WHO_AM_I = 0x70 -- mpu6500
MPU9250_WHO_AM_I = 0x71 -- mpu9250
ICM20608G_WHO_AM_I = 0xAF -- icm20608G
ICM20608D_WHO_AM_I = 0xAE -- icm20608D

MPU6XXX_ACCEL_SEN = 16384
MPU6XXX_GYRO_SEN = 1310

MPU60X0_TEMP_SEN = 340
MPU60X0_TEMP_OFFSET = 36.5

MPU6500_TEMP_SEN = 333.87
MPU6500_TEMP_OFFSET = 21

---MPU6XXX所用地址
MPU6XXX_RA_ACCEL_XOUT_H = 0x3B
MPU6XXX_RA_ACCEL_XOUT_L = 0x3C
MPU6XXX_RA_ACCEL_YOUT_H = 0x3D
MPU6XXX_RA_ACCEL_YOUT_L = 0x3E
MPU6XXX_RA_ACCEL_ZOUT_H = 0x3F
MPU6XXX_RA_ACCEL_ZOUT_L = 0x40
MPU6XXX_RA_TEMP_OUT_H = 0x41
MPU6XXX_RA_TEMP_OUT_L = 0x42
MPU6XXX_RA_GYRO_XOUT_H = 0x43
MPU6XXX_RA_GYRO_XOUT_L = 0x44
MPU6XXX_RA_GYRO_YOUT_H = 0x45
MPU6XXX_RA_GYRO_YOUT_L = 0x46
MPU6XXX_RA_GYRO_ZOUT_H = 0x47
MPU6XXX_RA_GYRO_ZOUT_L = 0x48

MPU6XXX_RA_SMPLRT_DIV = 0x19 -- 陀螺仪采样率，典型值：0x07(125Hz)
MPU6XXX_RA_CONFIG = 0x1A -- 低通滤波频率，典型值：0x06(5Hz)
MPU6XXX_RA_GYRO_CONFIG = 0x1B -- 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
MPU6XXX_RA_ACCEL_CONFIG = 0x1C -- 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
MPU6XXX_RA_FIFO_EN = 0x23 -- fifo使能
MPU6XXX_RA_INT_PIN_CFG = 0x37 -- int引脚有效电平
MPU6XXX_RA_INT_ENABLE = 0x38 -- 中断使能
MPU6XXX_RA_USER_CTRL = 0x6A
MPU6XXX_RA_PWR_MGMT_1 = 0x6B -- 电源管理，典型值：0x00(正常启用)
MPU6XXX_RA_PWR_MGMT_2 = 0x6C
MPU6XXX_RA_WHO_AM_I = 0x75
-- 器件ID检测
function mpu6xxx_check()
    i2c.send(i2cid, MPU6XXX_ADDRESS_AD0_LOW, MPU6XXX_RA_WHO_AM_I) -- 读器件地址
    sys.wait(50)
    revData = i2c.recv(i2cid, MPU6XXX_ADDRESS_AD0_LOW, 1)
    if revData:byte() ~= nil then
        i2cslaveaddr = MPU6XXX_ADDRESS_AD0_LOW
    else
        i2c.send(i2cid, MPU6XXX_ADDRESS_AD0_HIGH, MPU6XXX_RA_WHO_AM_I) -- 读器件地址
        sys.wait(50)
        revData = i2c.recv(i2cid, MPU6XXX_ADDRESS_AD0_HIGH, 1)
        if revData:byte() ~= nil then
            i2cslaveaddr = MPU6XXX_ADDRESS_AD0_HIGH
        else
            log.info("i2c", "Can't find device")
            return 1
        end
    end
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_WHO_AM_I) -- 读器件地址
    sys.wait(50)
    revData = i2c.recv(i2cid, i2cslaveaddr, 1)
    log.info("Device i2c address is#:", revData:toHex())
    if revData:byte() == MPU6050_WHO_AM_I then
        deviceid = MPU6050_WHO_AM_I
        log.info("Device i2c id is: MPU6050")
    elseif revData:byte() == MPU6500_WHO_AM_I then
        deviceid = MPU6500_WHO_AM_I
        log.info("Device i2c id is: MPU6500")
    elseif revData:byte() == MPU9250_WHO_AM_I then
        deviceid = MPU9250_WHO_AM_I
        log.info("Device i2c id is: MPU9250")
    elseif revData:byte() == ICM20608G_WHO_AM_I then
        deviceid = ICM20608G_WHO_AM_I
        log.info("Device i2c id is: ICM20608G")
    elseif revData:byte() == ICM20608D_WHO_AM_I then
        deviceid = ICM20608D_WHO_AM_I
        log.info("Device i2c id is: ICM20608D")
    else
        log.info("i2c", "Can't find device")
        return 1
    end
    return 0
end

-- 器件初始化
function mpu6xxx_init()
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_PWR_MGMT_1, 0x80}) -- 复位
    sys.wait(100)
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_PWR_MGMT_1, 0x00}) -- 唤醒
    sys.wait(100)
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_SMPLRT_DIV, 0x07}) -- 陀螺仪采样率，典型值：0x07(125Hz)
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_CONFIG, 0x06}) -- 低通滤波频率，典型值：0x06(5Hz)
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_GYRO_CONFIG, 0x18}) -- 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_ACCEL_CONFIG, 0x01}) -- 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
    -- i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_FIFO_EN, 0x00})--关闭fifo
    -- i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_INT_ENABLE, 0x00})--关闭所有中断
    -- i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_USER_CTRL, 0x00})--I2C主模式关闭
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_PWR_MGMT_1, 0x01}) -- 设置x轴的pll为参考
    i2c.send(i2cid, i2cslaveaddr, {MPU6XXX_RA_PWR_MGMT_2, 0x00}) -- 加速度计与陀螺仪开启
    log.info("i2c init_ok")
end
-- 获取温度的原始数据
function mpu6xxx_get_temp_raw()
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_TEMP_OUT_H) -- 获取的地址
    buffer = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取2字节
    temp = string.unpack(">h", buffer)
    -- log.info("get_temp_raw type: "..type(buffer).." hex: "..buffer:toHex().." temp: "..temp)
    return temp or 0
end
-- 获取加速度计的原始数据
function mpu6xxx_get_accel_raw()
    accel = {
        x = nil,
        y = nil,
        z = nil
    }
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_ACCEL_XOUT_H) -- 获取的地址
    x = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    accel.x = string.unpack(">h", x)
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_ACCEL_YOUT_H) -- 获取的地址
    y = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    accel.y = string.unpack(">h", y)
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_ACCEL_ZOUT_H) -- 获取的地址
    z = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    accel.z = string.unpack(">h", z)
    -- log.info("get_accel_raw: x="..x:toHex().." y="..y:toHex().." z="..z:toHex())
    return accel or 0
end
-- 获取陀螺仪的原始数据
function mpu6xxx_get_gyro_raw()
    gyro = {
        x = nil,
        y = nil,
        z = nil
    }
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_GYRO_XOUT_H) -- 获取的地址
    x = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    gyro.x = string.unpack(">h", x)
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_GYRO_YOUT_H) -- 获取的地址
    y = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    gyro.y = string.unpack(">h", y)
    i2c.send(i2cid, i2cslaveaddr, MPU6XXX_RA_GYRO_ZOUT_H) -- 获取的地址
    z = i2c.recv(i2cid, i2cslaveaddr, 2) -- 获取6字节
    gyro.z = string.unpack(">h", z)
    return gyro or 0
end
-- 获取温度的原始数据
function mpu6xxx_get_temp()
    temp = nil
    tmp = mpu6xxx_get_temp_raw()
    if deviceid == MPU6050_WHO_AM_I then
        temp = tmp / MPU60X0_TEMP_SEN + MPU60X0_TEMP_OFFSET
    else
        temp = tmp / MPU6500_TEMP_SEN + MPU6500_TEMP_OFFSET
    end
    return temp
end
-- 以HMC5883L的x轴正方向为机体x轴正方向，y轴正方向为机体y轴负向，建立机体坐标系
-- 根据惯导模块上MPU6050的安装方向，MPU6050 x轴正方向为模块z轴正向，MPU6050 y轴正向为x轴负方向，MPU6050 z轴正方向为y轴正向

-- 获取加速度计的数据，单位: g
function mpu6xxx_get_accel()
    accel = {
        x = nil,
        y = nil,
        z = nil
    }
    tmp = mpu6xxx_get_accel_raw()
    accel.z = tmp.x  / MPU6XXX_ACCEL_SEN
    accel.x = -tmp.y  / MPU6XXX_ACCEL_SEN
    accel.y = tmp.z  / MPU6XXX_ACCEL_SEN
    return accel
end
-- 获取陀螺仪的数据，单位: deg / s
function mpu6xxx_get_gyro()
    gyro = {
        x = nil,
        y = nil,
        z = nil
    }
    tmp = mpu6xxx_get_gyro_raw()
    gyro.z = tmp.x * 10 / MPU6XXX_GYRO_SEN
    gyro.x = -tmp.y * 10 / MPU6XXX_GYRO_SEN
    gyro.y = tmp.z * 10 / MPU6XXX_GYRO_SEN
    return gyro
end
function check_i2c()
    if i2c.setup(i2cid, i2c.FAST) ~= i2c.FAST then
        log.error("testI2c.init", "fail")
        return
    end
    if mpu6xxx_check() ~= 0 then
        return
    end
end

function init_mpu6050()
    log.info("init_mpu6050 start---------------")
    sys.wait(1000)
    check_i2c()
    mpu6xxx_init()
    log.info("init_mpu6050 end---------------")
end
