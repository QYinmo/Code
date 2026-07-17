import subprocess

command = "sleep 2 && ros2 launch drone_cartographer cartographer.launch.py"

# 在新开一个 Bash 终端中运行命令
subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])
