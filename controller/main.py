# 导入socket模块
import socket
from os import system
from controller import Controller

# 创建一个UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 设置套接字为非阻塞模式
sock.setblocking(False)

if __name__ == "__main__":
    system('color 3e')
    system('title W801控制程序')
    local_port = int(input("请输入本机监听端口(默认777):") or "777")
    remote_port = int(input("请输入W801设备监听端口(默认11451):") or "11451")
    # 绑定本机端口
    sock.bind(('0.0.0.0', local_port))

    print("等待W801设备上线...")
    # 无限循环，监听端口
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            # 如果数据是"Online"字符串
            if data.decode() == "Online":
                target_ip = addr[0]
                print("W801设备上线, ip地址:" + target_ip)
                break
        except:
            # 如果发生异常，继续循环
            continue
    if target_ip:
        controller = Controller(sock,target_ip,remote_port)
        controller.mainloop()

# 关闭套接字
sock.close()
