import threading
import queue
import time
import cv2


class CameraStream:
    """
    一个线程安全的摄像头视频流类，用于解决缓冲区滞后问题。
    它在一个单独的线程中读取帧，并只保留最新的一帧在队列中。
    """

    def __init__(self, cam, queue_size=1):
        """
        初始化摄像头视频流。

        Args:
            cam: 摄像头
            queue_size (int): 队列的最大容量，设置为1可以确保只保留最新帧。
        """
        self.stream = cam
        if not self.stream.isOpened():
            raise IOError("无法打开摄像头或视频源")

        self.queue = queue.Queue(maxsize=queue_size)
        self.stopped = False
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True  # 守护线程，主程序退出时自动结束

    def start(self):
        """启动视频流读取线程。"""
        self.thread.start()
        print("摄像头读取线程已启动。")
        return self

    def update(self):
        """
        在后台线程中不断读取帧，并更新队列。
        """
        while not self.stopped:
            if not self.stream.isOpened():
                self.stop()
                break

            ret, frame = self.stream.read()
            if not ret:
                print("无法从摄像头读取帧，线程停止。")
                self.stop()
                break

            # 如果队列已满，说明有未处理的旧帧，将其丢弃
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass

            # 将最新帧放入队列
            self.queue.put(frame)

            # 这是一个小的优化，避免在极快帧率下CPU占用过高
            # 如果你的摄像头帧率很稳定，可以不加
            # time.sleep(0.001)

        print("摄像头读取线程已停止。")

    def read(self):
        """
        从队列中获取最新帧。这是主线程调用的方法。

        Returns:
            np.ndarray: 最新的图像帧。如果队列为空，此调用会阻塞。
        """
        return self.queue.get()

    def read_nowait(self):
        """
        非阻塞地从队列中获取最新帧。

        Returns:
            np.ndarray: 最新的图像帧。

        Raises:
            queue.Empty: 如果队列为空，则抛出此异常。
        """
        return self.queue.get_nowait()

    def is_running(self):
        """检查摄像头线程是否在运行。"""
        return not self.stopped

    def stop(self):
        """停止摄像头读取线程并释放资源。"""
        self.stopped = True
        self.stream.release()
        self.thread.join()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cam_stream = None
    try:
        # 1. 初始化并启动摄像头读取线程
        cam_stream = CameraStream(src=0).start()

        print("主程序开始处理图像...")

        while True:
            # 2. 从队列中非阻塞地获取最新帧
            #    使用 try...except 来处理队列为空的情况
            try:
                frame = cam_stream.read_nowait()
            except queue.Empty:
                # 队列为空时，稍作等待，避免CPU空转
                # 在此期间，主线程可以执行其他任务
                time.sleep(0.01)
                continue

            # 3. 在这里进行你的图像处理
            #    例如，将其转换为灰度图
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 4. 显示处理后的图像
            cv2.imshow("Original", frame)
            cv2.imshow("Processed (Grayscale)", gray_frame)

            # 5. 等待按键，并检查是否按下了 'q' 键
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except IOError as e:
        print(f"初始化摄像头失败: {e}")
    except Exception as e:
        print(f"程序运行中发生错误: {e}")
    finally:
        # 6. 确保在程序退出时安全地停止摄像头线程和释放资源
        if cam_stream and cam_stream.is_running():
            cam_stream.stop()
        print("主程序已退出。")