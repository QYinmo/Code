import numpy as np
SPEED = 35
HEIGHT = 150
SERVO_CAM_LEFT = 0
SERVO_CAM_RIGHT = 180

Y_A = -25
Y_B = -125
Y_C = -225
Y_D = -325
mission_points_list = []
mission_2_points_list = []
def P(x, y, z): return np.array([x * 50+25, -y * 175, z*35+90])


def point_arrange(y, z):
    for i in range(1, 4):
        mission_points_list.append(P(i, y, z))
    for i in range(3, 0, -1):
        mission_points_list.append(P(i, y, 1-z))


def change_dimension(y, z):
    mission_points_list.append(P(-1, y, 1-z))
    mission_points_list.append(P(-1, y+1, 1-z))


def mission2_arrange(id):
    mission_2_points_list.append(P(-1, 0, 1))  # 我直接后退
    if 1 <= id <= 6:
        # mission_2_points_list.append(P(-1, 0, 0))#第一路 A
        mission_2_points_list.append(mission_points_list[id-1])  # 直接过去不就完了
        mission_2_points_list.append(P(5, 0, 0))  # 都飞一样高算了
    elif 7 <= id <= 18:  # 第二路 BC共用
        mission_2_points_list.append(P(-1, 1, 0))
        mission_2_points_list.append(mission_points_list[id+1])
        mission_2_points_list.append(P(5, 1, 0))
    elif 19 <= id <= 24:  # 第三路 D
        mission_2_points_list.append(P(-1, 2, 0))
        mission_2_points_list.append(mission_points_list[id+3])
    mission_2_points_list.append(P(5, 2, 0))

point_arrange(0, 0)
change_dimension(0, 0)
point_arrange(1, 1)
point_arrange(1, 0)
change_dimension(1, 0)
point_arrange(2, 1)
mission2_arrange(19)
MISSION_POINT= np.array(mission_points_list)
MISSION_2_POINT = np.array(mission_2_points_list)
print(MISSION_2_POINT)