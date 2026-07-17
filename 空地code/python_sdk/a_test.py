from trash.usr_serial import Serial_station
import time

use_serial = True
if use_serial:
    try:
        # 初始化串口（接收长度设为最小，因为我们不关心接收）
        serial_terminal = Serial_station(
            device="cp2102", 
            baudrate=115200, 
            rx_length=2)  # 最小化接收缓冲区
        
        # 初始化发送数据 (14字节协议格式)
        terminal_com = [0xAA] + [10]*12 + [0xFF]  # 起始0xAA + 12数据 + 结束0xFF
        
        # 开始传输（只发送不处理接收）
        serial_terminal.start_transmit(terminal_com)
        
        # 测试循环 - 修改并发送数据
        for i in range(1,6):
            # 更新第二个数据字节(索引1)
            terminal_com[i] = i  
            
            print(f"发送数据包 {i}: {[hex(x) for x in terminal_com]}")
            time.sleep(1)
            
    except Exception as e:
        print(f"串口通信错误: {e}")
        # 可选的错误处理或回退逻辑