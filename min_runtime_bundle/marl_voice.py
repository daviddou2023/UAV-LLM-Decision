"""
MARL 语音引擎模块 - VoiceEngine v4.0 (服务化版)

更新:
1. [核心] AI推理优先走 HTTP 调用外部 whisper_server.py (模型常驻，重启秒可用)
2. [兼容] 若服务不可用，自动降级为本地加载 (行为与 v3.0 完全一致)
3. 录音架构、线程安全、shutdown 逻辑均未改动

架构:
- 服务模式: 两线程 (UI主线程 / 录音线程)，推理由外部服务承担
- 本地模式: 三线程 (UI主线程 / 录音线程 / AI推理常驻线程)

依赖:
- 服务模式: pip install pyaudio requests  (主程序侧无需 GPU)
- 本地模式: pip install pyaudio faster-whisper

用法:
- 先启动服务:  python whisper_server.py
- 再启动主程序: python marl_main.py --mode demo
- 若未启动服务，主程序会自动降级为本地加载 (首次约3分钟)

$$$$$$$$$$$本地降级模式首次加载需要等待最少3分钟语音模型的正常读取与编码$$$$$$$$$$$$$$$$$
"""
import threading
import queue
import time
import numpy as np
import requests as http_requests

# === PyAudio 延迟导入保护 ===
try:
    import pyaudio
    _HAS_PYAUDIO = True
except ImportError:
    _HAS_PYAUDIO = False


TACTICAL_PROMPT = (
    "空域拦截防御系统。开始任务，暂停，继续。"
    "全体发射，全体返航，返回基地。"
    "无人机，拦截机，主打，随动，列阵扯网，撞击拦截。"
    "敌机，巡飞弹，诱饵，高速突防，左翼，右翼，中路。"
    "警戒线，防线，场景一公里，场景三公里，场景五公里。"
)

TRANSCRIBE_OPTIONS = dict(
    beam_size=6,
    language="zh",
    vad_filter=True,
    vad_parameters=dict(min_silence_duration_ms=320, speech_pad_ms=180),
    condition_on_previous_text=False,
    temperature=0.0,
    initial_prompt=TACTICAL_PROMPT,
)


class VoiceEngine:
    # === 服务端配置 ===
    WHISPER_SERVER_URL = "http://127.0.0.1:5555"
    HEALTH_TIMEOUT = 2.0    # 探活超时 (秒)
    INFER_TIMEOUT = 15.0    # 推理超时 (秒)
    MIN_RECORD_SEC = 0.22

    def __init__(self, model_size="small", test_wav=None):
        """
        test_wav: 传入 WAV 文件路径则进入文件测试模式 (远程调试用)
                  例: VoiceEngine(test_wav="test_cmd.wav")
                  设为 None 则使用真实麦克风
        """
        self.result_queue = queue.Queue()
        self.task_queue = queue.Queue()

        self.is_ready = False
        self.model_size = model_size
        self.test_wav = test_wav

        # === 推理模式标记 ===
        self._use_server = False   # True=HTTP服务模式, False=本地模式
        self.model = None          # 本地模式才用

        # === 录音状态 (线程安全) ===
        self._stop_event = threading.Event()
        self._record_thread = None
        self._frames = []
        self._frames_lock = threading.Lock()

        # === PyAudio 初始化 ===
        if _HAS_PYAUDIO:
            self.audio = pyaudio.PyAudio()
        else:
            self.audio = None
        self.chunk = 1024
        self.format = pyaudio.paInt16 if _HAS_PYAUDIO else 8  # paInt16=8
        self.channels = 1
        self.rate = 16000
        self._stream = None
        self._stream_lock = threading.Lock()

        self.device_index = None
        if self.audio:
            try:
                info = self.audio.get_default_input_device_info()
                self.device_index = info['index']
                print(f"[VoiceEngine] 锁定麦克风: {info['name']}")
            except Exception as e:
                print(f"[VoiceEngine] 警告: 未检测到麦克风 ({e})")

        # === 启动 AI 常驻线程 ===
        self._running = True
        self.ai_thread = threading.Thread(target=self._ai_worker_loop, daemon=True)
        self.ai_thread.start()

    # ==========================================================
    # AI 推理常驻线程
    # ==========================================================
    def _ai_worker_loop(self):
        """
        启动时先探活 whisper_server，可用则走 HTTP；
        不可用则降级为本地加载 Whisper 模型。
        """

        # ---- 第一步: 尝试连接外部服务 ----
        try:
            resp = http_requests.get(
                f"{self.WHISPER_SERVER_URL}/health",
                timeout=self.HEALTH_TIMEOUT,
            )
            if resp.status_code == 200:
                health = resp.json()
                self._use_server = True
                self.is_ready = True
                model_name = health.get("model_name", "unknown")
                print(f"[VoiceEngine] ✓ 已连接 Whisper 服务 ({model_name})")
                self.result_queue.put(f"[SYS] 语音服务已连接({model_name})，按住 V 键开始指挥！")
        except Exception:
            pass

        # ---- 第二步: 服务不可用 → 本地加载 ----
        if not self._use_server:
            print("[VoiceEngine] 未检测到 Whisper 服务，降级为本地加载...")
            try:
                from faster_whisper import WhisperModel
                import torch
                if torch.cuda.is_available():
                    device, compute_type = "cuda", "float16"
                else:
                    device, compute_type = "cpu", "int8"

                print(f"[VoiceEngine] 正在常驻线程中加载 Whisper ({device})...")
                self.model = WhisperModel(self.model_size, device=device, compute_type=compute_type)
                self.is_ready = True
                print("[VoiceEngine] 模型加载完成，AI 守护线程进入待命状态！")
                self.result_queue.put("[SYS] 语音模型加载完毕，按住 V 键开始指挥！")
            except Exception as e:
                self.result_queue.put(f"[错误] 模型加载失败: {str(e)}")
                return

        # ---- 第三步: 主循环 - 等待音频任务 ----
        while self._running:
            try:
                audio_frames = self.task_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if audio_frames is None:
                break

            try:
                audio_data = b''.join(audio_frames)

                if self._use_server:
                    text = self._infer_via_server(audio_data)
                else:
                    text = self._infer_locally(audio_data)

                if text:
                    self.result_queue.put(text)
                else:
                    self.result_queue.put("[SYS] 未识别到有效指令")
            except Exception as e:
                self.result_queue.put(f"[错误] 语音解析异常: {str(e)}")

    # ==========================================================
    # 推理路径 A: HTTP 服务调用
    # ==========================================================
    def _infer_via_server(self, audio_data: bytes) -> str:
        """将 PCM 字节发送到 whisper_server，返回识别文本"""
        try:
            resp = http_requests.post(
                f"{self.WHISPER_SERVER_URL}/transcribe",
                data=audio_data,
                headers={"Content-Type": "application/octet-stream"},
                timeout=self.INFER_TIMEOUT,
            )
            if resp.status_code == 200:
                text = resp.json().get("text", "").strip()
                text = text.replace("。", "").replace("，", "").replace(".", "").strip()
                return text if text else None

            # 服务异常，尝试降级
            self.result_queue.put("[SYS] 语音服务异常，尝试本地降级...")
            self._try_fallback_to_local()
            return None
        except http_requests.exceptions.ConnectionError:
            self.result_queue.put("[SYS] 语音服务断开，尝试本地降级...")
            self._try_fallback_to_local()
            return None
        except Exception as e:
            return None

    def _try_fallback_to_local(self):
        """服务中途挂了 → 动态降级为本地模式"""
        if self.model is not None:
            # 本地模型已经加载过，直接切换
            self._use_server = False
            return

        try:
            from faster_whisper import WhisperModel
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"
            compute = "float16" if device == "cuda" else "int8"
            print(f"[VoiceEngine] 紧急本地加载 Whisper ({device})...")
            self.model = WhisperModel(self.model_size, device=device, compute_type=compute)
            self._use_server = False
            self.result_queue.put("[SYS] 已切换至本地语音模型")
        except Exception as e:
            self.result_queue.put(f"[错误] 本地降级失败: {e}")

    # ==========================================================
    # 推理路径 B: 本地推理
    # ==========================================================
    def _infer_locally(self, audio_data: bytes) -> str:
        """使用本地 Whisper 模型推理"""
        audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
        segments, _ = self.model.transcribe(audio_np, **TRANSCRIBE_OPTIONS)
        text = "".join([seg.text for seg in segments]).strip()
        text = text.replace("。", "").replace("，", "").replace(".", "").strip()
        return text if text else None

    # ==========================================================
    # 公开 API (UI 主线程调用)
    # ==========================================================
    def start_recording(self):
        """按下 V 键时调用"""
        if not self.is_ready:
            self.result_queue.put("[SYS] 语音模型尚未准备好，请稍候...")
            return

        # === 文件测试模式: 直接读 WAV 送推理，跳过录音 ===
        if self.test_wav:
            return  # 按下时不做事，松开时直接提交文件

        # 如果上一次录音线程还没退干净，先等一会，不然可能会卡死
        if self._record_thread and self._record_thread.is_alive():
            self._stop_event.set()
            self._record_thread.join(timeout=1.0)

        # 重置状态
        self._stop_event.clear()
        with self._frames_lock:
            self._frames = []

        # 打开音频流
        try:
            with self._stream_lock:
                self._stream = self.audio.open(
                    format=self.format,
                    channels=self.channels,
                    rate=self.rate,
                    input=True,
                    input_device_index=self.device_index,
                    frames_per_buffer=self.chunk,
                )
        except Exception as e:
            self.result_queue.put(f"[错误] 麦克风开启失败: {str(e)}")
            return

        # 启动录音线程
        self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._record_thread.start()

    def stop_recording(self):
        """松开 V 键时调用"""

        # === 文件测试模式 直接读 WAV 文件提交 ===
        if self.test_wav:
            try:
                import wave
                with wave.open(self.test_wav, 'rb') as wf:
                    raw = wf.readframes(wf.getnframes())
                # 把原始 PCM 字节按 chunk 切块，模拟真实录音帧
                chunk_bytes = self.chunk * 2  # 16bit = 2 bytes per sample
                frames = [raw[i:i+chunk_bytes] for i in range(0, len(raw), chunk_bytes)]
                print(f"[VoiceEngine] 测试模式: 读取 {len(raw)/self.rate:.1f}s 音频")
                self.task_queue.put(frames)
            except Exception as e:
                self.result_queue.put(f"[错误] 测试文件读取失败: {e}")
            return

        # === 真实录音模式 ===
        # 第一步: 通知录音线程停止
        self._stop_event.set()

        # === 第二步: 等录音线程彻底退出 ===
        if self._record_thread and self._record_thread.is_alive():
            self._record_thread.join(timeout=2.0)
        self._record_thread = None

        # === 第三步: 录音线程已死，现在安全关闭 stream ===
        with self._stream_lock:
            if self._stream:
                try:
                    self._stream.stop_stream()
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None

        # === 第四步: 提交音频给 AI 线程 ===
        with self._frames_lock:
            frames_copy = list(self._frames)
            self._frames = []

        min_frames = int(self.rate / self.chunk * self.MIN_RECORD_SEC)
        if len(frames_copy) < min_frames:
            self.result_queue.put("[SYS] 录音过短，已忽略")
            return

        self.task_queue.put(frames_copy)

    def get_result(self):
        """每帧调用 (非阻塞)"""
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None

    # ==========================================================
    # 录音线程
    # ==========================================================
    def _record_loop(self):
        """
        录音线程的唯一职责: 从 stream 读数据到 frames。
        """
        while not self._stop_event.is_set():
            with self._stream_lock:
                stream = self._stream
            if stream is None:
                break

            try:
                # exception_on_overflow=False 防止偶发的 buffer overflow 异常
                data = stream.read(self.chunk, exception_on_overflow=False)
                with self._frames_lock:
                    self._frames.append(data)
            except OSError:
                # stream 可能已被外部关闭 (通常不会走到这里，但作为防御)
                break
            except Exception:
                break

        # 线程结束，不做任何 stream 操作，自动退出

    # ==========================================================
    # 系统关闭
    # ==========================================================
    def shutdown(self):
        """程序退出时调用"""
        self._running = False

        # 1. 停止录音
        self._stop_event.set()
        if self._record_thread and self._record_thread.is_alive():
            self._record_thread.join(timeout=2.0)

        # 2. 关闭 stream
        with self._stream_lock:
            if self._stream:
                try:
                    self._stream.stop_stream()
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None

        # 3. 终止 PyAudio
        if self.audio:
            try:
                self.audio.terminate()
            except Exception:
                pass

        # 4. 通知 AI 线程退出
        self.task_queue.put(None)
