madgwick = {}

_G.sys = require "sys"

local deg2rad = math.pi / 180 -- 角度转弧度的系数

-- 定义 madgwick 梯度下降姿态解算算法新实例
function madgwick:new(dt, betaDef, gammaDef)
    local obj = {}
    obj.dt = dt or 120 -- 采样间隔时间，单位为 ms
    obj.beta = betaDef or 0.1 -- 梯度下降算法的参数
    obj.gamma = gammaDef or 0.01 -- 磁力计数据的权重
    obj.q0 = 1 -- 四元数的第一个分量
    obj.q1 = 0 -- 四元数的第二个分量
    obj.q2 = 0 -- 四元数的第三个分量
    obj.q3 = 0 -- 四元数的第四个分量
    setmetatable(obj, self) -- 设置实例的元表为类本身
    self.__index = self -- 设置元表的__index字段为类本身，使得实例可以访问类的方法
    return obj
end

-- 定义一个方法，用于计算两个向量的叉积
function madgwick:crossProduct(ax, ay, az, bx, by, bz)
    local cx = ay * bz - az * by
    local cy = az * bx - ax * bz
    local cz = ax * by - ay * bx
    return cx, cy, cz
end

-- 定义一个方法，用于计算两个向量的点积
function madgwick:dotProduct(ax, ay, az, bx, by, bz)
    local d = ax * bx + ay * by + az * bz
    return d
end

-- 定义一个方法，用于计算向量的模长
function madgwick:norm(ax, ay, az)
    local n = math.sqrt(ax * ax + ay * ay + az * az)
    return n
end

-- 定义一个方法，用于归一化向量
function madgwick:normalize(ax, ay, az)
    local n = self:norm(ax, ay, az)
    if n == 0 then
        return ax, ay, az
    end -- 避免除以零的情况
    ax = ax / n
    ay = ay / n
    az = az / n
    return ax, ay, az
end

-- 定义一个方法，用于更新四元数和姿态角（欧拉角）
function madgwick:updateIMU(ax, ay, az, gx, gy, gz, mx, my, mz)
    -- 将陀螺仪数据转换为弧度制
    gx = gx * deg2rad
    gy = gy * deg2rad
    gz = gz * deg2rad

    -- 归一化加速度计数据和磁力计数据
    ax, ay, az = self:normalize(ax, ay, az)
    mx, my, mz = self:normalize(mx, my, mz)

    -- 计算重力方向和加速度计测量方向之间的误差（参考论文公式（25））
    local ex1 = (ay * self.q3 - az * self.q2) + (ay * self.q1 - az * self.q0) *
                    (2 * self.q0 * self.q2 + 2 * self.q1 * self.q3) + (ay * self.q0 + az * self.q1) *
                    (2 * self.q0 * self.q3 - 2 * self.q1 * self.q2)
    local ey1 = (az * self.q1 - ax * self.q3) + (az * self.q0 - ax * self.q2) *
                    (2 * self.q0 * self.q2 + 2 * self.q1 * self.q3) + (az * self.q2 + ax * self.q3) *
                    (2 * self.q0 * self.q3 - 2 * self.q1 * self.q2)
    local ez1 = (ax * self.q2 - ay * self.q1) + (ax * self.q3 - ay * self.q0) *
                    (2 * self.q0 * self.q2 + 2 * self.q1 * self.q3) + (ax * self.q1 + ay * self.q2) *
                    (2 * self.q0 * self.q3 - 2 * self.q1 * self.q2)

    -- 根据当前的四元数，计算出参考方向上的磁场分量 hx, hy, hz（参考论文公式（34））
    local hx = mx * (0.5 - self.q2 * self.q2 - self.q3 * self.q3) + my * (self.q1 * self.q2 - self.q0 * self.q3) + mz *
                   (self.q1 * self.q3 + self.q0 * self.q2)
    local hy = mx * (self.q1 * self.q2 + self.q0 * self.q3) + my * (0.5 - self.q1 * self.q1 - self.q3 * self.q3) + mz *
                   (self.q2 * self.q3 - self.q0 * self.q1)
    local hz = mx * (self.q1 * self.q3 - self.q0 * self.q2) + my * (self.q2 * self.q3 + self.q0 * self.q1) + mz *
                   (0.5 - self.q1 * self.q1 - self.q2 * self.q2)

    -- 计算磁力计测量方向和参考方向之间的误差 ex, ey, ez（参考论文公式（45））
    local ex2 = my * hz - mz * hy
    local ey2 = mz * hx - mx * hz
    local ez2 = mx * hy - my * hx

    -- 将两个误差相加，得到总的误差
    local ex = ex1 + ex2
    local ey = ey1 + ey2
    local ez = ez1 + ez2

    -- 根据误差计算反馈项（参考论文公式（26）和（46））
    local feedbackx = 2 * ((self.beta * ex) + (self.gamma * ex)) -- 加速度计数据的反馈项乘以 beta，磁力计数据的反馈项乘以 gamma
    local feedbacky = 2 * ((self.beta * ey) + (self.gamma * ey)) -- 加速度计数据的反馈项乘以 beta，磁力计数据的反馈项乘以 gamma
    local feedbackz = 2 * ((self.beta * ez) + (self.gamma * ez)) -- 加速度计数据的反馈项乘以 beta，磁力计数据的反馈项乘以 gamma

    -- 将反馈项加到陀螺仪数据上，得到修正后的角速度
    gx = gx + feedbackx
    gy = gy + feedbacky
    gz = gz + feedbackz

    -- 根据修正后的角速度更新四元数（参考论文公式（12））
    local qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
    local qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
    local qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
    local qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

    -- 根据采样时间和一阶积分法更新四元数
    self.q0 = self.q0 + qDot1 * self.dt/ 1000
    self.q1 = self.q1 + qDot2 * self.dt/ 1000
    self.q2 = self.q2 + qDot3 * self.dt/ 1000
    self.q3 = self.q3 + qDot4 * self.dt/ 1000

    -- 归一化四元数，保证其模长为1
    local n = self:norm(self.q0, self.q1, self.q2, self.q3)
    if n == 0 then
        return
    end -- 避免除以零的情况
    self.q0 = self.q0 / n
    self.q1 = self.q1 / n
    self.q2 = self.q2 / n
    self.q3 = self.q3 / n

    -- 根据四元数计算姿态角（欧拉角），单位为度（参考论文公式（13））
    local roll =
        math.atan2(2 * (self.q0 * self.q1 + self.q2 * self.q3), 1 - 2 * (self.q1 * self.q1 + self.q2 * self.q2)) / deg2rad
    local pitch = math.asin(2 * (self.q0 * self.q2 - self.q3 * self.q1)) / deg2rad
    local yaw =
        math.atan2(2 * (self.q0 * self.q3 + self.q1 * self.q2), 1 - 2 * (self.q2 * self.q2 + self.q3 * self.q3)) / deg2rad

    -- 返回姿态角（欧拉角）
    return roll, pitch, yaw
end

return madgwick
