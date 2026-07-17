import cv2
import subprocess
import re


def get_camera_by_usb_location(usb_location):
    """
    通过USB位置锁定摄像头
    Args:
        usb_location: USB位置，如 "usb-0000:00:14.0-3.1"
    Returns:
        摄像头索引号
    """
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            devices = result.stdout.split('\n\n')
            
            for device_block in devices:
                if device_block.strip():
                    lines = device_block.strip().split('\n')
                    device_name = lines[0].strip()
                    
                    # 检查USB位置
                    if usb_location in device_name:
                        # 提取视频设备
                        for line in lines[1:]:
                            if '/dev/video' in line:
                                video_path = line.strip().split('\t')[-1]
                                index = int(video_path.replace('/dev/video', ''))
                                
                                # 验证摄像头是否可用
                                try:
                                    cap = cv2.VideoCapture(index)
                                    if cap.isOpened():
                                        cap.release()
                                        return index
                                except:
                                    continue
                
    except Exception as e:
        print(f"通过USB位置获取摄像头失败: {e}")
    
    return None


def get_sy_1080p_camera():
    """锁定 SY 1080P 摄像头"""
    camera_index = get_camera_by_usb_location("usb-0000:00:14.0-3.1")
    if camera_index is not None:
        return camera_index
    return None

def get_usb_2_0_camera():
    """锁定 USB 2.0 摄像头"""
    camera_index = get_camera_by_usb_location("usb-0000:00:14.0-3.2")
    if camera_index is not None:
        return camera_index
    return None

def get_all_cameras_info():
    """用于获取所有摄像头的详细信息"""
    cameras_info = []
    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                camera_info = {
                    'index': i,
                    'resolution': f"{width}x{height}",
                    'fps': fps,
                    'backend': cap.getBackendName()
                }
                cameras_info.append(camera_info)
                cap.release()
        except:
            continue
    
    return cameras_info

# 测试函数
def test_camera_locking():

    print("\n尝试锁定 SY 1080P 摄像头...")
    sy_camera = get_sy_1080p_camera()
    if sy_camera is not None:
        print(f" 成功锁定 SY 1080P 摄像头，索引: {sy_camera}")
        
        # 测试打开摄像头
        cap = cv2.VideoCapture(sy_camera)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  分辨率: {width}x{height}")
            cap.release()
    else:
        print(" 无法锁定 SY 1080P 摄像头")
    
    print("\n尝试锁定 USB 2.0 摄像头...")
    usb_camera = get_usb_2_0_camera()
    if usb_camera is not None:
        print(f" 成功锁定 USB 2.0 摄像头，索引: {usb_camera}")

        cap = cv2.VideoCapture(usb_camera)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  分辨率: {width}x{height}")
            cap.release()
    else:
        print(" 无法锁定 USB 2.0 摄像头")
    
    return sy_camera, usb_camera

# 使用示例
if __name__ == "__main__":
    sy_cam, usb_cam = test_camera_locking()
    get_all_cameras_info()
    print(f"\n=== 最终锁定结果 ===")
    print(f"SY 1080P 摄像头索引: {sy_cam}")
    print(f"USB 2.0 摄像头索引: {usb_cam}")
    # 实际使用示例
    if sy_cam is not None and usb_cam is not None:
        print("\n可以开始使用锁定的摄像头:")
        print(f"cap1 = cv2.VideoCapture({sy_cam})  # SY 1080P 摄像头")
        print(f"cap2 = cv2.VideoCapture({usb_cam})  # USB 2.0 摄像头")