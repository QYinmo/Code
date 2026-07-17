import matplotlib.pyplot as plt
import matplotlib.patches as patches
from trash.usr_serial import Serial_station
import time

# 创建图形和轴
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, 4)
ax.set_ylim(0, 3)
ax.axis('off')  # 隐藏坐标轴

# 自定义编号顺序
custom_order = [10, 9, 6, 5, 11, 8, 7, 4, 0, 1, 2, 3]

# 初始化文本对象字典
text_objects = {}

# 创建3行4列的网格，按照自定义顺序编号
positions = [(0, 0), (0, 1), (0, 2), (0, 3),
             (1, 0), (1, 1), (1, 2), (1, 3),
             (2, 0), (2, 1), (2, 2), (2, 3)]

for idx, (i, j) in enumerate(positions):
    # 计算位置（从左下角开始）
    x = j
    y = 2 - i
    
    # 绘制矩形边框
    rect = patches.Rectangle((x, y), 1, 1, linewidth=2,
                            edgecolor='black', facecolor='none')
    ax.add_patch(rect)
    
    # 添加编号标签（使用自定义顺序）
    region_num = custom_order[idx]
    ax.text(x + 0.5, y + 0.7, str(region_num),
            ha='center', va='center', fontsize=16, color='blue')
    
    # 添加可更新的文本对象（初始为空，红色）
    text_obj = ax.text(x + 0.5, y + 0.3, '',
                      ha='center', va='center', fontsize=12, color='red')
    text_objects[region_num] = text_obj  # 使用自定义编号作为键

def update_regions(data_dict):
    """根据给定的字典更新区域内容，键为自定义编号"""
    for region_num, text_obj in text_objects.items():
        if region_num in data_dict:
            text_obj.set_text(str(data_dict[region_num]))
        else:
            text_obj.set_text('')
    plt.draw()

def trans(number):
    """将数字转换为对应的文本"""
    classes = {
        0: 'vocano',
        1: 'mountain',
        2: 'lake',
        3: 'river',
        4: 'farmland',
        5: 'village',
        6: 'mudslide'
    }
    return classes.get(number, 'unknown')

# 初始显示
plt.tight_layout()
plt.ion()  # 开启交互模式
plt.show()

# 串口通信设置
use_serial = True

if use_serial:
    try:
        serial_terminal = Serial_station(device="cp2102", baudrate=115200, rx_length=14)
        
        buffer_to_region = {
            1:0, 2:1, 3:2, 4:3, 5:4, 6:5, 7:6, 8:7, 9:8, 10:9, 11:10, 12:11
        }

        # 初始化缓冲区
        terminal_rxbuffer = [170]+[10] * 12+[255]  # 12个数据位
        
        # 启动通信

        serial_terminal.start_transmit(terminal_rxbuffer)
        
        while True:
            # 检查是否有新数据
            if serial_terminal.is_listened:
                try:
                    update_data = {}
                    # 加锁读取数据
                    for buf_idx, region_num in buffer_to_region.items():
                            if 0 <= buf_idx-1 < len(terminal_rxbuffer):
                                num = terminal_rxbuffer[buf_idx-1]  # 索引从0开始
                                if num != 10:  # 跳过默认值
                                    update_data[region_num] = trans(num)
                    
                    print(f"更新数据: {terminal_rxbuffer}")
                    update_regions(update_data)
                except Exception as e:
                    print(f"数据处理错误: {e}")
                finally:
                    serial_terminal.is_listened = False
                
                
            plt.pause(0.05)  # 控制刷新率
                
    except Exception as e:
        print(f"串口通信错误: {e}")
        # 回退到示例数据
        update_regions({
            10: "A", 9: "B", 6: "C", 5: "D",
            11: "E", 8: "F", 7: "G", 4: "H",
            0: "I", 1: "J", 2: "K", 3: "L"
        })



