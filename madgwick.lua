madgwick = {}

_G.sys = require "sys"

local deg2rad = math.pi / 180 -- 角度转弧度的系数

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

-- 定义 madgwick 梯度下降姿态解算算法新实例
function madgwick:new(dt, betaDef, gammaDef)
    local obj = {}
    obj.dt = dt or 100 -- 采样间隔时间，单位为 ms
    obj.dt = obj.dt/1000
    obj.beta = betaDef or 0.1 -- 梯度下降算法的参数
    obj.q = {1,0,0,0} -- 初始化姿态四元数
    setmetatable(obj, self) -- 设置实例的元表为类本身
    self.__index = self -- 设置元表的__index字段为类本身，使得实例可以访问类的方法
    return obj
end

-- 定义目标方法，用于计算加速度计和磁力计的误差
function madgwick:objectiveFunction(q)
    local q_conj = quatConj(q) -- 四元数的共轭
    local g_est = quatProd(quatProd(q, self.g_ref), q_conj) -- 重力向量的估计值
    local m_est = quatProd(quatProd(q, self.m_ref), q_conj) -- 磁场向量的估计值
    local f1 = g_est[1] - self.g_sen[1] -- 加速度计误差的第一分量
    local f2 = g_est[2] - self.g_sen[2] -- 加速度计误差的第二分量
    local f3 = g_est[3] - self.g_sen[3] -- 加速度计误差的第三分量
    local f4 = m_est[1] - self.m_sen[1] -- 磁力计误差的第一分量
    local f5 = m_est[2] - self.m_sen[2] -- 磁力计误差的第二分量
    local f6 = m_est[3] - self.m_sen[3] -- 磁力计误差的第三分量
    return {f1, f2, f3, f4, f5, f6} -- 返回误差向量
end

-- 定义雅可比矩阵，用于计算目标函数的梯度
function madgwick:jacobianMatrix(mx, my, mz, q)
    local a, b, c, d = q[1], q[2], q[3], q[4]
    local j11 = -2 * d; local j12 = 2 * c; local j13 = -2 * b; local j14 = 2 * a;
    local j21 = 2 * c; local j22 = 2 * d; local j23 = 2 * a; local j24 = 2 * b;
    local j31 = 0; local j32 = -4 * b; local j33 = -4 * c; local j34 = 0;
    local j41 = -2 * b * mz + 2 * c * my; local j42 = -2 * a * mz + 2 * d * my;
    local j43 = -2 * a * my - 2 * d * mz; local j44= -2 * b * my + 2 * c * mz;
    local j51= -2 * a * mx + 2 * d * mz;local j52= -2 * b * mx + 2 * c * mz;
    local j53= -2 * b * mz - 2 * c * mx;local j54= -2 * a* mz + 2* d* mx;
    local j61= -2* c* mx + 2* b* my;local j62= -2* d* mx + 2* a* my;
    local j63= -4* a* mx + 4* d* my;local j64= -4* b* mx +4* c* my;

    return {{j11, j12, j13, j14}, {j21, j22, j23, j24}, {j31, j32, j33, j34},
            {j41, j42, j43, j44}, {j51, j52, j53, j54}, {j61, j62, j63, j64}}
end

-- 定义一个方法，用于更新四元数和姿态角（欧拉角）
function madgwick:update(ax, ay, az, gx, gy, gz, mx, my, mz)
    -- 如果加速度计或磁力计数据为零，则直接返回
    if ax == 0 or ay == 0 or az == 0 or mx == 0 or my == 0 or mz == 0 then
        return
    end
    local norm_a = math.sqrt(ax^2 + ay^2 + az^2)

    -- 归一化加速度计和磁力计数据
    ax = ax / norm_a
    ay = ay / norm_a
    az = az / norm_a
    local norm_m = math.sqrt(mx^2 + my^2 + mz^2)
    mx = mx / norm_m
    my = my / norm_m
    mz = mz / norm_m

    -- 定义重力向量和磁场向量在地理坐标系下的参考值
    self.g_ref = {0, 0, 0, 1}
    self.m_ref = {0, math.cos(0), math.sin(0), 0} -- 假设磁偏角为0

    -- 定义重力向量和磁场向量在传感器坐标系下的测量值
    self.g_sen = {0, ax, ay, az}
    self.m_sen = {0, mx, my, mz}

    -- 计算目标函数的值
    local f = self:objectiveFunction(self.q)

    -- 计算雅可比矩阵的值
    local j = self:jacobianMatrix(mx, my, mz, self.q)

    -- 计算目标函数的梯度
    local grad = {0, 0, 0, 0}
    for i = 1, 4 do
        for k = 1, 6 do
          grad[i] = grad[i] + j[k][i] * f[k]
        end
    end

    -- 归一化梯度
    local norm_grad = math.sqrt(grad[1]^2 + grad[2]^2 + grad[3]^2 + grad[4]^2)
    grad[1] = grad[1] / norm_grad
    grad[2] = grad[2] / norm_grad
    grad[3] = grad[3] / norm_grad
    grad[4] = grad[4] / norm_grad

    -- 计算陀螺仪的四元数变化率
    local q_dot_gyro = {0.5 * (-self.q[2] * gx - self.q[3] * gy - self.q[4] * gz),
    0.5 * (self.q[1] * gx + self.q[3] * gz - self.q[4] * gy),
    0.5 * (self.q[1] * gy - self.q[2] * gz + self.q[4] * gx),
    0.5 * (self.q[1] * gz + self.q[2] * gy - self.q[3] * gx)}

    -- 计算梯度下降的四元数变化率
    local q_dot_grad = {-self.beta * grad[1], -self.beta * grad[2], -self.beta * grad[3], -self.beta * grad[4]}

    -- 计算总的四元数变化率
    local q_dot = {q_dot_gyro[1] + q_dot_grad[1],
    q_dot_gyro[2] + q_dot_grad[2],
    q_dot_gyro[3] + q_dot_grad[3],
    q_dot_gyro[4] + q_dot_grad[4]}

    -- 更新四元数
    self.q[1] = self.q[1] + q_dot[1] * self.dt
    self.q[2] = self.q[2] + q_dot[2] * self.dt
    self.q[3] = self.q[3] + q_dot[3] * self.dt
    self.q[4] = self.q[4] + q_dot[4] * self.dt

    -- 归一化四元数
    local norm_q = quatNorm(self.q)
    self.q[1] = self.q[1] / norm_q
    self.q[2] = self.q[2] / norm_q
    self.q[3] = self.q[3] / norm_q
    self.q[4] = self.q[4] / norm_q

    -- 计算欧拉角
    local raw, pitch, yaw = quatToEuler(self.q)
    -- 返回欧拉角
    return raw, pitch, yaw
end

return madgwick
