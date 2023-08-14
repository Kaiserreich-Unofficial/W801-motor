mahony = {}

_G.sys = require "sys"

local deg2rad = math.pi / 180 -- 角度转弧度的系数
local twoKp = 2*0.5-- 比例增益 Kp
local twoKi = 2*0.01-- 积分增益 Ki
-- 按 Ki 缩放的积分误差项
local integralFBx = 0.0
local integralFBy = 0.0
local integralFBz = 0.0

-- 定义一个函数，用于计算两个四元数的乘积
local function quatProd(q1, q2)
    local a1, b1, c1, d1 = q1[1], q1[2], q1[3], q1[4]
    local a2, b2, c2, d2 = q2[1], q2[2], q2[3], q2[4]
    local a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    local b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    local c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    local d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
    return {a, b, c, d}
end

-- 定义一个函数，用于计算四元数的共轭
local function quatConj(q)
    local a, b, c, d = q[1], q[2], q[3], q[4]
    return {a, -b, -c, -d}
end

-- 定义一个函数，用于计算四元数的模长
local function quatNorm(q)
    local a, b, c, d = q[1], q[2], q[3], q[4]
    return math.sqrt(a^2 + b^2 + c^2 + d^2)
end

-- 定义一个函数，用于计算四元数的归一化
local function quatNormalize(q)
    local a, b, c, d = q[1], q[2], q[3], q[4]
    local n = quatNorm(q)
    return {a / n, b / n, c / n, d / n}
end

-- 定义一个函数，用于计算四元数对应的欧拉角（假设为Z-Y-X顺序）
local function quatToEuler(q)
    local a, b, c, d = q[1], q[2], q[3], q[4]
    local roll = math.atan(2 * (a * b + c * d) / (a^2 - b^2 - c^2 + d^2)) / deg2rad
    local pitch = math.asin(2 * (a * c - b * d)) / deg2rad
    local yaw = math.atan(2 * (a * d + b * c) / (a^2 + b^2 - c^2 - d^2)) / deg2rad
    return roll, pitch, yaw
end

-- 定义一个函数，将欧拉角向量转换为四元数向量
function mahony:Eulertoquat(euler)
    -- 从参数中提取欧拉角的分量
    local alpha, beta, gamma = euler[1], euler[2], euler[3]
    -- 计算绕x轴旋转alpha角度对应的四元数
    local qx = {math.cos(alpha/2), math.sin(alpha/2), 0, 0}
    -- 计算绕y轴旋转beta角度对应的四元数
    local qy = {math.cos(beta/2), 0, math.sin(beta/2), 0}
    -- 计算绕z轴旋转gamma角度对应的四元数
    local qz = {math.cos(gamma/2), 0, 0, math.sin(gamma/2)}
-- 将这三个四元数按照顺序相乘得到最终的四元数
    local q = quatProd(quatProd(qz, qy), qx)
    -- 返回结果四元数
    return q
end

-- 定义 mahony 梯度下降姿态解算算法新实例
function mahony:new(q0, dt)
    local obj = {}
    obj.dt = dt or 120 -- 采样间隔时间，单位为 ms
    obj.dt = obj.dt/1000
    obj.q = q0 or {1,0,0,0} -- 初始化姿态四元数

    setmetatable(obj, self) -- 设置实例的元表为类本身
    self.__index = self -- 设置元表的__index字段为类本身，使得实例可以访问类的方法
    return obj
end

-- 定义一个方法，用于更新四元数和姿态角（欧拉角）
function mahony:update(ax, ay, az, gx, gy, gz, mx, my, mz)
    local halfvx, halfvy, halfvz = nil, nil, nil
    local halfwx, halfwy, halfwz = nil, nil, nil
    local halfex, halfey, halfez = nil, nil, nil
    local qa, qb, qc = nil, nil, nil


    -- 如果加速度计或磁力计数据为零，则直接返回
    if ax == 0 or ay == 0 or az == 0 or mx == 0 or my == 0 or mz == 0 then
        return
    end
    -- 归一化加速度计和磁力计数据
    local norm_a = math.sqrt(ax^2 + ay^2 + az^2)
    ax = ax / norm_a
    ay = ay / norm_a
    az = az / norm_a
    local norm_m = math.sqrt(mx^2 + my^2 + mz^2)
    mx = mx / norm_m
    my = my / norm_m
    mz = mz / norm_m

    -- 计算重力方向的参考方向向量（NED坐标系）
    local hx = mx * self.q[1]^2 - 2 * self.q[1] * my * self.q[4] + 2 * self.q[1] * mz * self.q[3] + mx * self.q[2]^2 + 2 * self.q[2] * my * self.q[3] + 2 * self.q[2] * mz * self.q[4] - mx * self.q[3]^2 - mx * self.q[4]^2
    local hy = 2 * self.q[1] * mx * self.q[4] + my * self.q[1]^2 - 2 * self.q[1] * mz * self.q[2] + 2 * self.q[2] * mx * self.q[3] - my * self.q[2]^2 + my * self.q[3]^2 + 2 * self.q[3] * mz * self.q[4] - my * self.q[4]^2
    local bx = math.sqrt(hx^2 + hy^2)
    local bz = -2 * self.q[1] * mx * self.q[3] + 2 * self.q[1] * my * self.q[2] + mz * self.q[1]^2 + 2 * self.q[2] * mx * self.q[4] - mz * self.q[2]^2 + 2 * self.q[3] * my * self.q[4] - mz * self.q[3]^2 + 2 * self.q[4] * my * self.q[4] - mz * self.q[4]^2

    -- 估计重力和磁场方向
    halfvx = self.q[2] * self.q[4] - self.q[1] * self.q[3]
    halfvy = self.q[1] * self.q[2] + self.q[3] * self.q[4]
    halfvz = self.q[1]^2 - 0.5 + self.q[3]^2
    halfwx = bx * (0.5 - self.q[3]^2 - self.q[4]^2) + bz * (self.q[2]*self.q[4] - self.q[1]*self.q[3])
    halfwy = bx * (self.q[2]*self.q[3] - self.q[1]*self.q[4]) + bz * (self.q[1]*self.q[2] + self.q[3]*self.q[4])
    halfwz = bx * (self.q[1]*self.q[3] + self.q[2]*self.q[4]) + bz * (0.5 - self.q[2]^2 - self.q[3]^2)

    -- 误差 = 估计方向&测量的场矢量方向之和
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

    if twoKi > 0 then
        integralFBx = integralFBx + twoKi * halfex * self.dt
        integralFBy = integralFBy + twoKi * halfey * self.dt
        integralFBz = integralFBz + twoKi * halfez * self.dt
        gx = gx + integralFBx
        gy = gy + integralFBy
        gz = gz + integralFBz
    else
        integralFBx = 0.0
        integralFBy = 0.0
        integralFBz = 0.0
    end
    -- 应用比例反馈
    gx = gx + twoKp * halfex
    gy = gy + twoKp * halfey
    gz = gz + twoKp * halfez

    -- 积分四元数变化率
    gx = gx * (0.5 * self.dt)
    gy = gy * (0.5 * self.dt)
    gz = gz * (0.5 * self.dt)
    qa = self.q[1]
    qb = self.q[2]
    qc = self.q[3]
    self.q[1] = self.q[1] + (-qb * gx - qc * gy - self.q[4] * gz)
    self.q[2] = self.q[2] + (qa * gx + qc * gz - self.q[4] * gy)
    self.q[3] = self.q[3] + (qa * gy - qb * gz + self.q[4] * gx)
    self.q[4] = self.q[4] + (qa * gz + qb * gy - qc * gx)

    -- 归一化四元数
    local norm_q = quatNormalize(self.q)

    -- 计算欧拉角
    local raw, pitch, yaw = quatToEuler(self.q)

    -- 返回欧拉角
    return raw, pitch, yaw
end

return mahony
