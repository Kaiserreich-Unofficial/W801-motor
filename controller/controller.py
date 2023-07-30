# 导入tkinter模块
import tkinter
import tkinter.ttk as ttk

# 定义一个名为Controller的类，继承自tkinter.Tk类
class Controller(tkinter.Tk):
    # 定义类的初始化方法
    def __init__(self,sock,ip,port):
        # 调用父类的初始化方法
        super().__init__()
        # 设置窗口的标题、大小和图标
        self.title("W801开环控制程序(按下Tab键调速)")
        self.geometry("300x200")
        # 创建四个按钮控件，分别显示Shift和WASD的状态
        self.w_button = tkinter.Button(self, text="W", bg="white", fg="black", state="disabled")
        self.w_button.grid(row=1, column=1, padx=10, pady=10, ipadx=24, ipady=20)
        self.a_button = tkinter.Button(self, text="A", bg="white", fg="black", state="disabled")
        self.a_button.grid(row=2, column=0, padx=10, pady=10, ipadx=24, ipady=20)
        self.s_button = tkinter.Button(self, text="S", bg="white", fg="black", state="disabled")
        self.s_button.grid(row=2, column=1, padx=10, pady=10, ipadx=24, ipady=20)
        self.d_button = tkinter.Button(self, text="D", bg="white", fg="black", state="disabled")
        self.d_button.grid(row=2, column=2, padx=10, pady=10, ipadx=24, ipady=20)
        # 创建一个标签控件，显示载具的状态
        self.status_label = tkinter.Label(self, text="停止", font=("Arial", 16))
        self.status_label.grid(row=3, column=1)
        # 创建一个滑动条控件，显示载具的速度，并绑定一个名为speed_change的回调函数
        self.speed_scale = ttk.Scale(self, from_=1, to=100, orient="horizontal", command=self.speed_change,value=50)
        self.speed_scale.grid(row=4, column=1)
        # 使用grid_remove方法，使其默认不显示
        self.speed_scale.grid_remove()
        # 创建一个全局数组，保存当前按下的按键
        self.keys_pressed = []
        # 定义一个全局变量，表示是否按下了Tab键
        self.tab_pressed = False
        # 定义初始速度 50
        self.speed = 50
        # 使用bind方法，绑定键盘事件和函数
        # 使用lambda表达式，根据按键的状态，更新数组和状态
        self.bind("<w>", lambda event: self.update_array_and_status("W", True))
        self.bind("<KeyRelease-w>", lambda event: self.update_array_and_status("W", False))
        self.bind("<a>", lambda event: self.update_array_and_status("A", True))
        self.bind("<KeyRelease-a>", lambda event: self.update_array_and_status("A", False))
        self.bind("<s>", lambda event: self.update_array_and_status("S", True))
        self.bind("<KeyRelease-s>", lambda event: self.update_array_and_status("S", False))
        self.bind("<d>", lambda event: self.update_array_and_status("D", True))
        self.bind("<KeyRelease-d>", lambda event: self.update_array_and_status("D", False))
        # 绑定Tab键盘事件，并定义一个函数来处理Tab键的按下和松开
        self.bind("<Tab>", self.tab_down)
        self.bind("<KeyRelease-Tab>", self.tab_up)

        # 使用grid_columnconfigure和grid_rowconfigure方法，设置网格的权重，使按钮居中显示
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)

        # 定义传入sock,ip和port参数
        self.sock = sock
        self.ip = ip
        self.port = port
        # 定义上次命令缓存
        self.last_order = None

    # 定义一个函数，用于处理滑动条的变化
    def speed_change(self,value):
        # 获取滑动条的当前值，并转换为整数
        self.speed = int(float(value))
        # 在标签上显示载具的速度
        self.status_label.config(text=f"速度：{self.speed}")

    # 定义一个函数，根据数组的内容，更新按钮和标签的状态
    def update_status(self):
        # 遍历所有按钮，如果对应的按键在数组中，就改变按钮的颜色和状态，否则就恢复默认
        for button in [self.w_button, self.a_button, self.s_button, self.d_button]:
            if button.cget("text") in self.keys_pressed:
                button.config(bg="green", fg="white", state="normal")
            else:
                button.config(bg="white", fg="black", state="disabled")
        # 根据数组的内容，判断载具的状态，并改变标签的文本
        if "W" in self.keys_pressed and "A" in self.keys_pressed:
            self.status_label.config(text="前进并左转")
            order = 'wa'+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "W" in self.keys_pressed and "D" in self.keys_pressed:
            self.status_label.config(text="前进并右转")
            order = 'wd'+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "S" in self.keys_pressed and "A" in self.keys_pressed:
            self.status_label.config(text="后退并右转")
            order = 'sa'+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "S" in self.keys_pressed and "D" in self.keys_pressed:
            self.status_label.config(text="后退并左转")
            order = 'sd'+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "W" in self.keys_pressed:
            self.status_label.config(text="前进")
            order = 'w '+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "S" in self.keys_pressed:
            self.status_label.config(text="后退")
            order = 's '+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "A" in self.keys_pressed:
            self.status_label.config(text="左转")
            order = 'a '+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        elif "D" in self.keys_pressed:
            self.status_label.config(text="右转")
            order = 'd '+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))
        else:
            self.status_label.config(text="停止")
            order = 'ws'+str(self.speed)
            if not order == self.last_order:
                self.last_order = order
                self.sock.sendto(order.encode(), (self.ip, self.port))

    # 定义一个函数，根据按键的状态，更新数组和状态
    def update_array_and_status(self,key,pressed):
        # 如果按下了按键，就把按键添加到数组中，否则就从数组中移除
        if pressed:
            if key not in self.keys_pressed:
                self.keys_pressed.append(key)
        else:
            if key in self.keys_pressed:
                self.keys_pressed.remove(key)

        # 如果按下了Tab键，就根据A和D键的状态，调整滑动条的大小
        if self.tab_pressed:
            # 如果按下了A键，就把滑动条的值减少1
            if "A" in self.keys_pressed:
                self.speed_scale.set(self.speed_scale.get() - 1)
            # 如果按下了D键，就把滑动条的值增加1
            if "D" in self.keys_pressed:
                self.speed_scale.set(self.speed_scale.get() + 1)
        # 否则，就调用更新状态的函数
        else:
            self.update_status()

    # 定义一个函数，用于处理Tab键的按下和松开
    def tab_down(self,event):
        # 设置全局变量为True，并显示滑动条控件
        self.tab_pressed = True
        self.status_label.config(text=f"速度：{self.speed}")
        self.speed_scale.grid()

    def tab_up(self,event):
        # 设置全局变量为False，并隐藏滑动条控件，并更新状态
        self.tab_pressed = False
        self.speed_scale.grid_remove()
        self.update_status()
