import cv2
import numpy as np
import pygame
import time
import sys
import platform


class CameraApp:
    def __init__(self):
        pygame.init()
        self.screen = None
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 24)
        self.image_counter = 0
        self.auto_mode = False
        self.auto_speed = 30
        self.frame_count = 0
        self.setup_window()

        # 初始化摄像头
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

    def setup_window(self):
        """安全初始化窗口"""
        self.screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
        pygame.display.set_caption("摄像头控制 v1.0")

        # Windows置顶功能
        if platform.system() == 'Windows':
            self.set_window_topmost()

    def set_window_topmost(self):
        """Windows专用置顶功能"""
        try:
            import ctypes
            hwnd = pygame.display.get_wm_info().get('window')
            if hwnd:
                ctypes.windll.user32.SetWindowPos(
                    hwnd, -1,  # HWND_TOPMOST
                    0, 0, 0, 0,
                    0x0001 | 0x0002  # SWP_NOMOVE | SWP_NOSIZE
                )
        except Exception as e:
            print(f"置顶失败: {str(e)}")

    def process_frame(self):
        """获取并处理摄像头帧"""
        ret, frame = self.cap.read()
        if not ret:
            return None

        # 转换颜色空间 BGR -> RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)  # 旋转90度适配Pygame坐标
        return pygame.surfarray.make_surface(frame)

    def draw_ui(self):
        """绘制UI控件"""
        # 半透明背景
        overlay = pygame.Surface((300, 150), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 128))
        self.screen.blit(overlay, (10, 10))

        # 状态文本
        status_text = [
            f"Model: {'Auto' if self.auto_mode else 'Maner'}",
            f"Save_conut: {self.image_counter}",
            "Space: Maner_Save",
            "A_Key: Shift_Auto",
            "ESC: ESC"
        ]

        for i, text in enumerate(status_text):
            text_surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(text_surface, (20, 20 + i * 30))

    def save_image(self, frame):
        """保存当前帧为图片"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"capture_{timestamp}_{self.image_counter}.jpg"

        # 转换回OpenCV格式保存
        cv_frame = cv2.cvtColor(
            pygame.surfarray.array3d(frame),
            cv2.COLOR_RGB2BGR
        )
        cv_frame = np.rot90(cv_frame, -1)
        cv2.imwrite(filename, cv_frame)

        self.image_counter += 1
        print(f"已保存: {filename}")

    def run(self):
        """主运行循环"""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        frame = self.process_frame()
                        if frame is not None:
                            self.save_image(frame)
                    elif event.key == pygame.K_a:
                        self.auto_mode = not self.auto_mode

            # 获取并显示摄像头画面
            frame_surface = self.process_frame()
            if frame_surface is not None:
                # 缩放适应窗口
                scaled_frame = pygame.transform.scale(
                    frame_surface,
                    (self.screen.get_width(), self.screen.get_height())
                )
                self.screen.blit(scaled_frame, (0, 0))

                # 自动模式处理
                if self.auto_mode:
                    self.frame_count += 1
                    if self.frame_count >= self.auto_speed:
                        self.frame_count = 0
                        self.save_image(frame_surface)

            # 绘制UI
            self.draw_ui()
            pygame.display.flip()
            self.clock.tick(30)  # 限制30FPS

        # 退出清理
        self.cap.release()
        pygame.quit()
        sys.exit()


if __name__ == '__main__':
    try:
        app = CameraApp()
        app.run()
    except Exception as e:
        print(f"程序崩溃: {str(e)}")
        if 'app' in locals():
            app.cap.release()
        pygame.quit()
