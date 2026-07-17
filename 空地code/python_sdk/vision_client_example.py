#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉检测服务客户端 - 简洁版
提供简单的API函数获取检测结果
"""

import json

import requests

# 服务器配置
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5432
BASE_URL = f"http://{SERVER_HOST}:{SERVER_PORT}"


def get_detection_results(block=False, timeout=30):
    """
    从视觉检测服务获取结果

    Args:
        block (bool): 是否阻塞等待新结果
            - False: 立即返回最新结果
            - True: 等待新结果后返回
        timeout (int): 阻塞等待的超时时间(秒)，仅在block=True时有效

    Returns:
        dict: 检测结果字典，包含以下字段：
            - success (bool): 是否成功获取结果
            - error (str): 错误信息（如果有）
            - timestamp (str): 时间戳
            - has_detection (bool): 是否检测到目标
            - detected_objects (list): 检测到的目标列表 （每个目标包含x,y,class_id,class_name,confidence）
            - class_counts (dict): 类别统计 （key: 类别名称, value: 数量）
            - fps (float): 帧率

    Example:
        # 立即获取最新结果
        result = get_detection_results()

        # 等待新结果（10秒超时）
        result = get_detection_results(block=True, timeout=10)

        # 检查结果
        if result['success'] and result['has_detection']:
            for obj in result['detected_objects']:
                print(f"检测到: {obj['class_name']}, 置信度: {obj['confidence']:.2f}")
    """

    try:
        if block:
            # 阻塞等待新结果
            response = requests.get(
                f"{BASE_URL}/result_block", params={"timeout": timeout}
            )
        else:
            # 立即获取最新结果
            response = requests.get(f"{BASE_URL}/result")

        if response.status_code == 200:
            data = response.json()
            data["success"] = True
            return data

        elif response.status_code == 503:
            return {
                "success": False,
                "error": "Detection service is not running",
                "has_detection": False,
                "detected_objects": [],
                "class_counts": {},
                "fps": 0.0,
            }

        elif response.status_code == 408:
            return {
                "success": False,
                "error": "Timeout waiting for new result",
                "has_detection": False,
                "detected_objects": [],
                "class_counts": {},
                "fps": 0.0,
            }

        else:
            return {
                "success": False,
                "error": f"HTTP {response.status_code}: {response.text}",
                "has_detection": False,
                "detected_objects": [],
                "class_counts": {},
                "fps": 0.0,
            }

    except requests.RequestException as e:
        return {
            "success": False,
            "error": f"Network error: {str(e)}",
            "has_detection": False,
            "detected_objects": [],
            "class_counts": {},
            "fps": 0.0,
        }
