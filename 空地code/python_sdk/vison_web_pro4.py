import base64
import json
import threading
import time
from datetime import datetime
import queue

import cv2
import numpy as np
from flask import Flask, jsonify, render_template_string, request
from flask_socketio import SocketIO, emit
from loguru import logger
from ultralytics import YOLO
from vision.Vision_plus import (
    HighPrecisionFPS,
    debug_imshow,
    find_red_area,
    get_area_in_frame,
    get_ROI,
    vision_debug,
    set_manual_exporsure,
    set_cam_autowb,
    open_camera_plus
)
from vision.cam_stream import CameraStream

# Initialize Flask app and SocketIO
app = Flask(__name__)
# app.config["SECRET_KEY"] = "public"  # no authentication needed
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
latest_results = {
    "timestamp": None,
    "has_detection": False,
    "detected_objects": [],
    "class_counts": {},
    "fps": 0.0,
    "frame_base64": None,
}

detection_running = False
detection_thread = None
cap = None
model = None
timer = None

# Event for blocking result endpoint
result_event = threading.Event()
result_condition = threading.Condition()

# Configuration
# Update this path as needed
MODEL_PATH = r"/home/n1/workplace/Drone_maindev/python_sdk/robocup_best.pt"
CONF_THRES = 0.7


def initialize_camera_and_model():
    """Initialize camera and YOLO model"""
    global cap, cam_stream, model, timer

    try:
        cap = open_camera_plus()
        set_cam_autowb(cap, True)
        set_manual_exporsure(cap, -6.5)
        cam_stream = CameraStream(cap, 2).start()
        model = YOLO(MODEL_PATH)
        timer = HighPrecisionFPS()

        vision_debug(True)
        logger.info("Camera and model initialized successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize camera and model: {e}")
        return False


def detect_objects_in_frame(img, model):
    """Detect objects in a single frame"""
    try:
        results = model.predict(
            source=img,
            show=False,
            conf=CONF_THRES,
            save=False,
            verbose=False,
            stream=True,
            device="cpu",
            iou=0.3
        )
        processed_results = []
        final_results = []  # 用于存储最终结果
        annotated_frame = img.copy()

        if results is not None:
            for result in results:
                if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                    boxes = result.boxes.cpu().numpy()
                    normalized_xy = boxes.xywhn[:, :2]
                    classes = boxes.cls
                    confs = boxes.conf
                    annotated_frame = result.plot()  # 自动绘制boxes/labels
                    debug_imshow(annotated_frame, "Result")
                    for i in range(len(classes)):
                        processed_results.append([
                            normalized_xy[i][0],       # x坐标
                            normalized_xy[i][1],       # y坐标
                            int(classes[i]),            # 类别
                            float(confs[i])            # 置信度
                        ])
                    processed_results = np.array(processed_results)
                    cls_results = processed_results
                    max_conf_idx = np.argmax(cls_results[:, 3])
                    best_result = cls_results[max_conf_idx]
                    final_results.append(best_result)
                    final_results = [list(row) for row in final_results]
                    for res in final_results:
                        print(
                            f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")

        return True, final_results, annotated_frame

    except Exception as e:
        logger.error(f"Detection failed: {e}")
        return False, [], img


def frame_to_base64(frame):
    """Convert frame to base64 string for web transmission"""
    try:
        _, buffer = cv2.imencode(".jpg", frame)
        frame_base64 = base64.b64encode(buffer).decode("utf-8")
        return frame_base64
    except Exception as e:
        logger.error(f"Failed to encode frame: {e}")
        return None


def detection_loop():
    """Main detection loop running in background thread"""
    global cap, latest_results, detection_running, cam_stream, model, timer

    logger.info("Detection loop started")

    while detection_running:
        try:
            if cap is None or model is None:
                time.sleep(1)
                continue
            try:
                frame = cam_stream.read_nowait()
                frame = get_ROI(frame, (100, 10, 440, 440))
            except queue.Empty:
                logger.warning("Failed to read frame from camera")
                time.sleep(0.01)
                continue

            # Detect objects
            has_detection, final_results, annotated_frame = (
                detect_objects_in_frame(frame, model)
            )

            # Calculate FPS
            current_fps = timer.fps()
            timer.reset()
            logger.info(f"fps:{current_fps}")

            # Convert frame to base64
            # debug_imshow(annotated_frame, "Result")

            # Update latest results
            latest_results.update(
                {
                    "timestamp": datetime.now().isoformat(),
                    "has_detection": has_detection and len(final_results) > 0,
                    "final_results": final_results,
                    "fps": current_fps,
                }
            )

            # Notify waiting threads of new results
            with result_condition:
                result_condition.notify_all()

            # Emit real-time data to WebSocket clients
            socketio.emit(
                "detection_update",
                {
                    "timestamp": latest_results["timestamp"],
                    "has_detection": latest_results["has_detection"],
                    "final_results": final_results,
                    "fps": current_fps,
                },
            )

            time.sleep(0.05)  # Small delay to prevent overwhelming

        except Exception as e:
            logger.error(f"Error in detection loop: {e}")
            time.sleep(1)

    logger.info("Detection loop stopped")


@app.route("/")
def index():
    """Main page with basic information"""
    return jsonify(
        {
            "service": "Vision Detection Service",
            "status": "running" if detection_running else "stopped",
            "endpoints": {
                "GET /result": "Get latest detection results (fast, no image)",
                "POST /result": "Get latest detection results (fast, no image)",
                "GET /result_block": "Wait for and get new detection results (timeout parameter supported)",
                "POST /result_block": "Wait for and get new detection results (timeout parameter supported)",
                "GET /monitor": "Real-time monitoring page",
                "POST /start": "Start detection service",
                "POST /stop": "Stop detection service",
            },
        }
    )


@app.route("/result", methods=["GET", "POST"])
def get_results():
    """Get latest detection results"""
    if not detection_running:
        return jsonify({"error": "Detection service is not running"}), 503
    # Remove frame_base64 for faster API response
    return jsonify(latest_results)


@app.route("/result_block", methods=["GET", "POST"])
def get_results_blocking():
    """Get latest detection results, wait for new result if needed"""
    timeout = request.args.get(
        "timeout", 30, type=int)  # Default 30 seconds timeout

    if not detection_running:
        return jsonify({"error": "Detection service is not running"}), 503

    # Wait for new result
    with result_condition:
        if result_condition.wait(timeout=timeout):
            return jsonify(latest_results)
        else:
            return jsonify({"error": "Timeout waiting for new result"}), 408


@app.route("/start", methods=["POST"])
def start_detection():
    """Start the detection service"""
    global detection_running, detection_thread

    if detection_running:
        return jsonify(
            {
                "status": "already_running",
                "message": "Detection service is already running",
            }
        )

    if not initialize_camera_and_model():
        return jsonify(
            {"status": "error", "message": "Failed to initialize camera and model"}
        ), 500

    detection_running = True
    detection_thread = threading.Thread(target=detection_loop, daemon=True)
    detection_thread.start()

    logger.info("Detection service started")
    return jsonify(
        {"status": "started", "message": "Detection service started successfully"}
    )


@app.route("/stop", methods=["POST"])
def stop_detection():
    """Stop the detection service"""
    global detection_running, cap, cam_stream

    detection_running = False
    if cam_stream and cam_stream.is_running():
        cam_stream.stop()
        logger.info("主程序已退出。")
    if cap is not None:
        cap.release()
        cv2.destroyAllWindows()

    logger.info("Detection service stopped")
    return jsonify(
        {"status": "stopped", "message": "Detection service stopped successfully"}
    )


@app.route("/monitor")
def monitor():
    """Real-time monitoring page"""
    monitor_html = """
<!DOCTYPE html>
<html>
<head>
    <title>视觉检测监控系统</title>
    <meta charset="UTF-8">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { background-color: #2c3e50; color: white; padding: 20px; border-radius: 5px; margin-bottom: 20px; }
        .content { display: flex; gap: 20px; }
        .left-panel { flex: 2; }
        .right-panel { flex: 1; }
        .card { background-color: white; padding: 20px; border-radius: 5px; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .status { padding: 10px; border-radius: 3px; margin-bottom: 10px; }
        .status.active { background-color: #d4edda; color: #155724; }
        .status.inactive { background-color: #f8d7da; color: #721c24; }
        #videoFrame { max-width: 100%; height: auto; border: 2px solid #ddd; border-radius: 5px; }
        .detection-item { background-color: #e9f7ff; padding: 10px; margin: 5px 0; border-radius: 3px; border-left: 4px solid #007bff; }
        .log-container { max-height: 300px; overflow-y: auto; background-color: #f8f9fa; padding: 10px; border-radius: 3px; font-family: monospace; font-size: 12px; }
        .log-entry { margin: 2px 0; }
        .log-entry.info { color: #0066cc; }
        .log-entry.error { color: #cc0000; }
        .log-entry.warning { color: #ff6600; }
        .fps-display { font-size: 18px; font-weight: bold; color: #007bff; }
        .controls { margin-bottom: 20px; }
        .btn { padding: 10px 20px; margin: 5px; border: none; border-radius: 3px; cursor: pointer; }
        .btn-primary { background-color: #007bff; color: white; }
        .btn-danger { background-color: #dc3545; color: white; }
        .btn:hover { opacity: 0.8; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎯 视觉检测监控系统</h1>
            <p>实时目标检测监控系统</p>
        </div>

        <div class="controls">
            <button class="btn btn-primary" onclick="startDetection()">启动检测</button>
            <button class="btn btn-danger" onclick="stopDetection()">停止检测</button>
        </div>

        <div class="content">
            <div class="left-panel">
                <div class="card">
                    <h3>📹 实时视频流</h3>
                    <img id="videoFrame" src="" alt="无视频信号" style="display: none;">
                    <p id="noVideoMessage">无视频信号，请启动检测服务查看实时画面</p>
                </div>

                <div class="card">
                    <h3>🎯 检测结果</h3>
                    <div id="detectionResults">
                        <p>暂无检测结果...</p>
                    </div>
                </div>
            </div>

            <div class="right-panel">
                <div class="card">
                    <h3>📊 系统状态</h3>
                    <div id="systemStatus" class="status inactive">系统未激活</div>
                    <div class="fps-display">帧率: <span id="fpsValue">0.0</span></div>
                    <p><strong>最后更新:</strong> <span id="lastUpdate">从未</span></p>
                </div>

                <div class="card">
                    <h3>📈 统计信息</h3>
                    <div id="classStats">
                        <p>无统计数据</p>
                    </div>
                </div>

                <div class="card">
                    <h3>📝 活动日志</h3>
                    <div id="logContainer" class="log-container">
                        <div class="log-entry info">[信息] 监控系统已初始化</div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();

        function addLog(message, type = 'info') {
            const logContainer = document.getElementById('logContainer');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.className = `log-entry ${type}`;
            logEntry.textContent = `[${timestamp}] ${message}`;
            logContainer.appendChild(logEntry);
            logContainer.scrollTop = logContainer.scrollHeight;

            // Keep only last 100 log entries
            while (logContainer.children.length > 100) {
                logContainer.removeChild(logContainer.firstChild);
            }
        }

        function updateVideoFrame(frameBase64) {
            const videoFrame = document.getElementById('videoFrame');
            const noVideoMessage = document.getElementById('noVideoMessage');

            if (frameBase64) {
                videoFrame.src = 'data:image/jpeg;base64,' + frameBase64;
                videoFrame.style.display = 'block';
                noVideoMessage.style.display = 'none';
            } else {
                videoFrame.style.display = 'none';
                noVideoMessage.style.display = 'block';
            }
        }

        function updateDetectionResults(data) {
            const resultsDiv = document.getElementById('detectionResults');

            if (data.has_detection && data.detected_objects.length > 0) {
                let html = '';
                data.detected_objects.forEach(obj => {
                    html += `
                        <div class="detection-item">
                            <strong>${obj.class_name}</strong><br>
                            Position: (${obj.x.toFixed(2)}, ${obj.y.toFixed(2)})<br>
                            Confidence: ${(obj.confidence * 100).toFixed(1)}%
                        </div>
                    `;
                });
                resultsDiv.innerHTML = html;
            } else {
                resultsDiv.innerHTML = '<p>未检测到目标</p>';
            }
        }

        function updateClassStats(classCounts) {
            const statsDiv = document.getElementById('classStats');

            if (Object.keys(classCounts).length > 0) {
                let html = '';
                for (const [className, count] of Object.entries(classCounts)) {
                    html += `<p><strong>${className}:</strong> ${count}</p>`;
                }
                statsDiv.innerHTML = html;
            } else {
                statsDiv.innerHTML = '<p>无统计数据</p>';
            }
        }

        function startDetection() {
            fetch('/start', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    addLog(data.message, data.status === 'error' ? 'error' : 'info');
                })
                .catch(error => {
                    addLog('Failed to start detection: ' + error, 'error');
                });
        }

        function stopDetection() {
            fetch('/stop', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    addLog(data.message, 'info');
                    document.getElementById('systemStatus').className = 'status inactive';
                    document.getElementById('systemStatus').textContent = '系统未激活';
                })
                .catch(error => {
                    addLog('Failed to stop detection: ' + error, 'error');
                });
        }

        socket.on('detection_update', function(data) {
            // Update system status
            const statusDiv = document.getElementById('systemStatus');
            statusDiv.className = 'status active';
            statusDiv.textContent = '系统运行中';

            // Update FPS
            document.getElementById('fpsValue').textContent = data.fps.toFixed(1);

            // Update last update time
            document.getElementById('lastUpdate').textContent = new Date(data.timestamp).toLocaleTimeString();

            // Update video frame
            updateVideoFrame(data.frame_base64);

            // Update detection results
            updateDetectionResults(data);

            // Update class statistics
            updateClassStats(data.class_counts);

            // Add log entry for detections
            if (data.has_detection) {
                const objectCount = data.detected_objects.length;
                addLog(`检测到 ${objectCount} 个目标`, 'info');
            }
        });

        socket.on('connect', function() {
            addLog('已连接到服务器', 'info');
        });

        socket.on('disconnect', function() {
            addLog('与服务器断开连接', 'warning');
            document.getElementById('systemStatus').className = 'status inactive';
            document.getElementById('systemStatus').textContent = '系统未激活';
        });
    </script>
</body>
</html>
    """
    return render_template_string(monitor_html)


def auto_start_detection():
    """Automatically start detection service when web service starts"""
    global detection_running, detection_thread

    logger.info("正在自动启动检测服务...")

    # Try to start detection service
    if initialize_camera_and_model():
        detection_running = True
        detection_thread = threading.Thread(target=detection_loop, daemon=True)
        detection_thread.start()
        logger.info("检测服务已自动启动")
        return True
    else:
        logger.warning("自动启动检测服务失败，请手动启动")
        return False


# Auto-start detection service when module is imported
auto_start_detection()

if __name__ == "__main__":
    logger.info("启动视觉检测Web服务...")

    # Ensure detection service is running
    if not detection_running:
        logger.info("检测服务未运行，尝试重新启动...")
        auto_start_detection()

    try:
        logger.info("Web服务已启动: http://0.0.0.0:5432")
        logger.info("监控页面: http://0.0.0.0:5432/monitor")
        logger.info("API端点: http://0.0.0.0:5432/result")
        socketio.run(app, host="0.0.0.0", port=5432, debug=False)
    except KeyboardInterrupt:
        logger.info("正在关闭服务...")
        detection_running = False
        if cam_stream and cam_stream.is_running():
            cam_stream.stop()
        if cap is not None:
            cap.release()
            cv2.destroyAllWindows()
        logger.info("服务已安全关闭")
