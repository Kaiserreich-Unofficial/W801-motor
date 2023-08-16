madgwick = {}

_G.sys = require "sys"

local deg2rad = math.pi / 180 -- 角度转弧度的系数

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
    local declinationAngle = 0.0404
    local roll = math.atan2(q[1]*q[2] + q[3]*q[4], 0.5 - q[2]^2 - q[3]^2) / deg2rad
    local pitch = math.asin(-2.0 * (q[2]*q[4] - q[1]*q[3])) / deg2rad
    local yaw = math.atan2(q[2]*q[3] + q[1]*q[4], 0.5 - q[3]^2 - q[4]^2)
    if yaw < 0 then yaw = yaw+2*math.pi
    elseif yaw > 2*math.pi then yaw = yaw-2*math.pi end
    yaw = (2*math.pi - yaw + declinationAngle) / deg2rad
    return roll, pitch, yaw
end

local function vec_cross(A,B)
    local cross = {nil,nil,nil}
    cross[1] = A[2] * B[3] - A[3] * B[2]
	cross[2] = A[3] * B[1] - A[1] * B[3]
	cross[3] = A[1] * B[2] - A[2] * B[1]
    return cross
end

function madgwick:init_q(accel,mag)
    -- 向下为负加速度计测量值
    local norm_a = math.sqrt(accel.x^2 + accel.y^2 + accel.z^2)
    accel.x = accel.x / norm_a
    accel.y = accel.y / norm_a
    accel.z = accel.z / norm_a
    -- 磁强计并不完全垂直于 "向下"，因此也不完全是 "向北"
    local norm_m = math.sqrt(mag.x^2 + mag.y^2 + mag.z^2)
    mag.x = mag.x / norm_m
    mag.y = mag.y / norm_m
    mag.z = mag.z / norm_m
    local D = {-accel.x,-accel.y,-accel.z}
    local m = {mag.x,mag.y,mag.z}
    -- 计算东方向
    local E = vec_cross(D,m)
    -- 标准化正东向量
    local norm_e = math.sqrt(E[1]^2 + E[2]^2 + E[3]^2)
    E[1] = E[1] / norm_e
    E[2] = E[2] / norm_e
    E[3] = E[3] / norm_e
    -- 计算北方向
    local N = vec_cross(D,E)
    -- 根据旋转矩阵 A=(N|D|E)计算欧拉参数（四元数）
    local Trace = N[1] + D[2] + E[3]
    local a = {Trace,N[1],D[2],E[3]}
    local e = {nil,nil,nil,nil}
    -- 寻找最大欧拉角的下标
    local k = 1
    for i=2,4 do
        if a[i] > a[k] then
            k = i
        end
    end

    e[k] = math.sqrt(1 + 2 * a[k] - Trace)/2

    if k == 1 then
        e[2] = (D[3] - E[2]) / (4 * e[1])
		e[3] = (E[1] - N[3]) / (4 * e[1])
		e[4] = (N[2] - D[1]) / (4 * e[1])
    elseif k==2 then
        e[1] = (D[3] - E[2]) / (4 * e[2])
		e[3] = (D[1] + N[2]) / (4 * e[2])
		e[4] = (E[1] + N[3]) / (4 * e[2])
    elseif k==3 then
        e[1] = (E[1] - N[3]) / (4 * e[3])
        e[2] = (D[1] + N[2]) / (4 * e[3])
        e[4] = (E[2] + D[3]) / (4 * e[3])
    elseif k==4 then
        e[1] = (N[2] - D[1]) / (4 * e[4])
        e[2] = (E[1] + N[3]) / (4 * e[4])
        e[3] = (E[2] + D[3]) / (4 * e[4])
    end
    -- 反转四元数旋转
	local q0 = e[1]
	local q1 = -e[2]
	local q2 = -e[3]
	local q3 = -e[4]
    self.q = {q0,q1,q2,q3}
end

-- 定义 madgwick 梯度下降姿态解算算法新实例
function madgwick:new(q0, betaDef)
    local obj = {}
    obj.q = q0 or {1,0,0,0} -- 初始化姿态四元数
    obj.beta = betaDef or 0.6
    setmetatable(obj, self) -- 设置实例的元表为类本身
    self.__index = self -- 设置元表的__index字段为类本身，使得实例可以访问类的方法
    return obj
end

-- 定义一个方法，用于更新四元数和姿态角（欧拉角）
function madgwick:update(ax, ay, az, gx, gy, gz, mx, my, mz, dt)
    dt = dt / 1000

    -- 如果加速度计或磁力计数据为零，则直接返回
    if ax == 0 or ay == 0 or az == 0 or mx == 0 or my == 0 or mz == 0 then
        return
    end

    local qDot1 = 0.5 * (-self.q[2] * gx - self.q[3] * gy - self.q[4] * gz)
	local qDot2 = 0.5 * (self.q[1] * gx + self.q[3] * gz - self.q[4] * gy)
	local qDot3 = 0.5 * (self.q[1] * gy - self.q[2] * gz + self.q[4] * gx)
	local qDot4 = 0.5 * (self.q[1] * gz + self.q[2] * gy - self.q[3] * gx)


    -- 归一化加速度计和磁力计数据
    local norm_a = math.sqrt(ax^2 + ay^2 + az^2)
    ax = ax / norm_a
    ay = ay / norm_a
    az = az / norm_a
    local norm_m = math.sqrt(mx^2 + my^2 + mz^2)
    mx = mx / norm_m
    my = my / norm_m
    mz = mz / norm_m

    -- Auxiliary variables to avoid repeated arithmetic
	local _2q0mx = 2.0 * self.q[1] * mx
	local _2q0my = 2.0 * self.q[1] * my
	local _2q0mz = 2.0 * self.q[1] * mz
	local _2q1mx = 2.0 * self.q[2] * mx
	local _2q0 = 2.0 * self.q[1]
	local _2q1 = 2.0 * self.q[2]
	local _2q2 = 2.0 * self.q[3]
	local _2q3 = 2.0 * self.q[4]
	local _2q0q2 = 2.0 * self.q[1] * self.q[3]
	local _2q2q3 = 2.0 * self.q[3] * self.q[4]
	local q0q0 = self.q[1]^2
	local q0q1 = self.q[1] * self.q[2]
	local q0q2 = self.q[1] * self.q[3]
	local q0q3 = self.q[1] * self.q[4]
	local q1q1 = self.q[2]^2
	local q1q2 = self.q[2] * self.q[3]
	local q1q3 = self.q[2] * self.q[4]
	local q2q2 = self.q[3]^2
	local q2q3 = self.q[3] * self.q[4]
	local q3q3 = self.q[4]^2

    -- 计算重力方向的参考方向向量（NED坐标系）
    local hx = mx * q0q0 - _2q0my * self.q[4] + _2q0mz * self.q[3] + mx * q1q1 + _2q1 * my * self.q[3] + _2q1 * mz * self.q[4] - mx * q2q2 - mx * q3q3
    local hy = _2q0mx * self.q[4] + my * q0q0 - _2q0mz * self.q[2] + _2q1mx * self.q[3] - my * q1q1 + my * q2q2 + _2q2 * mz * self.q[4] - my * q3q3
    local _2bx = math.sqrt(hx^2 + hy^2)
    local _2bz = -_2q0mx * self.q[3] + _2q0my * self.q[2] + mz * q0q0 + _2q1mx * self.q[4] - mz * q1q1 + _2q2 * my * self.q[4] - mz * q2q2 + mz * q3q3
    local _4bx = 2.0 * _2bx
    local _4bz = 2.0 * _2bz
    -- 梯度修正算法矫正步长
    local s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.q[3] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q[4] + _2bz * self.q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q[3] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    local s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q[2] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.q[4] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q[3] + _2bz * self.q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q[4] - _4bz * self.q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    local s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q[3] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.q[3] - _2bz * self.q[1]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q[2] + _2bz * self.q[4]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q[1] - _4bz * self.q[3]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    local s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.q[4] + _2bz * self.q[2]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q[1] + _2bz * self.q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    -- 步长标准化
    local norm_s = math.sqrt(s0^2+s1^2+s2^2+s3^2)
    s0 = s0 / norm_s
    s1 = s1 / norm_s
    s2 = s2 / norm_s
    s3 = s3 / norm_s
    -- 应用反馈步长
    qDot1 = qDot1 - self.beta * s0
    qDot2 = qDot2 - self.beta * s1
    qDot3 = qDot3 - self.beta * s2
    qDot4 = qDot4 - self.beta * s3

    -- 积分四元数的变化率，得出四元数
    self.q[1] = self.q[1] + qDot1 * dt
	self.q[2] = self.q[2] + qDot2 * dt
	self.q[3] = self.q[3] + qDot3 * dt
	self.q[4] = self.q[4] + qDot4 * dt

    -- 归一化四元数
    local norm_q = quatNormalize(self.q)

    -- 计算欧拉角
    local raw, pitch, yaw = quatToEuler(norm_q)

    -- 返回欧拉角
    return raw, pitch, yaw
end

return madgwick
