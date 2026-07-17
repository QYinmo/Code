import os

def get_map_publisher() -> int:
    # 执行命令并获取输出
    output = os.popen("ros2 topic info /map").read()
    
    # 将输出按行分割
    lines = output.split("\n")
    
    # 遍历每一行，查找包含 "Publisher count" 的行
    for line in lines:
        if "Publisher count" in line:
            # 提取 "Publisher count" 的值
            count_str = line.split(":")[1].strip()
            return int(count_str)
    
    # 如果未找到 "Publisher count" 行，返回 0 或其他默认值
    return 0

# 测试函数
publisher_count = get_map_publisher()
print(f"Publisher count: {publisher_count}")
