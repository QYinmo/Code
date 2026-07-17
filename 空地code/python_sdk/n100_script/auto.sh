###########################################
#自动化启动两个 Python 脚本（server_ros.py 和 mission_run.py）
CODE_PATH="$HOME/workplace/Drone_maindev/python_sdk"
###########################################
# >>> conda initialize >>>
__conda_setup="$('/home/$USER/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/$USER/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/$USER/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/$USER/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
conda activate env00
# >>> fishros initialize >>>
source /opt/ros/foxy/setup.bash 
source ~/workplace/package/install/setup.bash
# <<< fishros initialize <<<
#gnome-terminal -- bash -c "source ~/.bashrc && python ~/trash_test/test1.py ; exec bash"
#gnome-terminal -- bash -c "source ~/.bashrc && python ~/trash_test/test2.py ; exec bash"
sleep 5

# Open new terminal and run server_ros.py
gnome-terminal -- bash -c "
__conda_setup=\"\$('/home/$USER/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)\"
if [ \$? -eq 0 ]; then
    eval \"\$__conda_setup\"
else
    if [ -f \"/home/$USER/miniconda3/etc/profile.d/conda.sh\" ]; then
        . \"/home/$USER/miniconda3/etc/profile.d/conda.sh\"
    else
        export PATH=\"/home/$USER/miniconda3/bin:\$PATH\"
    fi
fi
unset __conda_setup
conda activate env00
source /opt/ros/foxy/setup.bash
source ~/workplace/package/install/setup.bash
cd $CODE_PATH
python $CODE_PATH/server_ros.py
exec bash"

sleep 6

# Open new terminal and run mission_run.py
gnome-terminal -- bash -c "
__conda_setup=\"\$('/home/$USER/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)\"
if [ \$? -eq 0 ]; then
    eval \"\$__conda_setup\"
else
    if [ -f \"/home/$USER/miniconda3/etc/profile.d/conda.sh\" ]; then
        . \"/home/$USER/miniconda3/etc/profile.d/conda.sh\"
    else
        export PATH=\"/home/$USER/miniconda3/bin:\$PATH\"
    fi
fi
unset __conda_setup
conda activate env00
source /opt/ros/foxy/setup.bash
source ~/workplace/package/install/setup.bash
cd $CODE_PATH
python $CODE_PATH/mission_run.py
exec bash"

