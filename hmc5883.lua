--HMC5883L磁力计
_G.sys = require "sys"

i2cid = 0 --i2cid

addr = 0x1e

Config_a     = 0x00    --配置寄存器a:设置的数据输出速率和测量配置  0 11 100 00  0X70
Config_b     = 0x01    --配置寄存器b：设置装置的增益              001 00000    0Xe0
mode         = 0x02    --模式寄存器: 默认单一测量模式01，连续00           000000 00    0X00/0x01
Msb_x        = 0x03    --x 高位数据输出
Lsb_x        = 0x04    --x 低位数据输出
Msb_y        = 0x07    --x 高位数据输出
Lsb_y        = 0x08    --x 低位数据输出
Msb_z        = 0x05    --x 高位数据输出
Lsb_z        = 0x06    --x 低位数据输出
status       = 0x09    --  状态寄存器    0x00
recogn_a     = 0x0a    --  识别寄存器a   0x48
recogn_b     = 0x0b    --  识别寄存器b   0x34
recogn_c     = 0x0c    --  识别寄存器c   0x33

--写数据
local function I2C_Write_Byte(regAddress,val,val2)
    i2c.send(i2cid, addr, {regAddress,val,val2})

end

--读取单个字节
local function I2C_Read_Byte(regAddress)
    i2c.send(i2cid, addr, regAddress)
    local rdstr = i2c.recv(i2cid, addr, 1)
    --log.info("rdstr:toHex()",rdstr:toHex())
    return rdstr:byte(1)--变成10进制数据
end

--读取多个字节
local function I2C_Read_Bytes(regAddress,cnt)
    i2c.send(i2cid, addr, regAddress)
    local rdstr = i2c.recv(i2cid, addr, cnt)
    --log.info("rdstr:toHex()-------",rdstr:toHex())
    return rdstr
end

function hmc5883l_init()

    I2C_Write_Byte(Config_a,0x70)  --写配置a寄存器数据
    I2C_Write_Byte(Config_b,0x20)  --写配置b寄存器数据  增益660
    I2C_Write_Byte(mode,0x00)      --写模式寄存器数据
end

function hmc5883l_read()

    local hx=I2C_Read_Byte(Msb_x)
    local lx=I2C_Read_Byte(Lsb_x)
    local x_data=hx*256+lx

    local hy=I2C_Read_Byte(Msb_y)
    local ly=I2C_Read_Byte(Lsb_y)
    local y_data=hy*256+ly

    local hz=I2C_Read_Byte(Msb_z)
    local lz=I2C_Read_Byte(Lsb_z)
    local z_data=hz*256+lz

    if(x_data>32768)  then
        x_data= -(0xFFFF - x_data + 1)
    end

    if(y_data>32768)  then
        y_data = -(0xFFFF - y_data + 1)
    end

    if(z_data>32768)  then
        z_data = -(0xFFFF - z_data+ 1)
    end

    --以HMC5883L的x轴正方向为机体x轴正方向，y轴正方向为机体y轴负向，建立机体坐标系
    --根据惯导模块上HMC5883L的安装方向，y轴和z轴正方向需取反
    y_data = -y_data
    z_data = -z_data

    local Angle= math.atan2(y_data,x_data) --单位：弧度(rad)

    return Angle,x_data,y_data,z_data
end
