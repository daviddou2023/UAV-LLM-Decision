"""
Whisper 常驻推理服务 - whisper_server.py
用途: 模型只加载一次，常驻 GPU 显存，主程序随便重启都秒连可用。

启动:
    python whisper_server.py                    # 默认 small 模型
    python whisper_server.py --model medium     # 用 medium 模型
    python whisper_server.py --port 5555        # 指定端口

依赖:
    pip install faster-whisper flask torch
"""
import argparse
import numpy as np
from flask import Flask, request, jsonify

app = Flask(__name__)
model = None
model_name = "unknown"

# === 战术词汇引导 (与 VoiceEngine 保持一致) ===
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


@app.route("/health", methods=["GET"])
def health():
    """探活接口，VoiceEngine 启动时会调用"""
    return jsonify({"status": "ok", "model_loaded": model is not None, "model_name": model_name})


@app.route("/transcribe", methods=["POST"])
def transcribe():
    """
    接收原始 16-bit PCM 字节 (16kHz, mono)，返回识别文本。
    请求体: raw bytes (application/octet-stream)
    响应: {"text": "识别结果"}
    """
    pcm_bytes = request.data
    if not pcm_bytes:
        return jsonify({"text": "", "error": "empty audio"}), 400

    audio_np = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0

    if len(audio_np) < 4800:  # 不足 0.3 秒
        return jsonify({"text": "", "error": "too short"})

    try:
        segments, _ = model.transcribe(audio_np, **TRANSCRIBE_OPTIONS)
        text = "".join(seg.text for seg in segments).strip()
        text = text.replace("。", "").replace("，", "").replace(".", "").strip()
        return jsonify({"text": text})
    except Exception as e:
        return jsonify({"text": "", "error": str(e)}), 500


def main():
    global model, model_name

    parser = argparse.ArgumentParser(description="Whisper 常驻推理服务")
    parser.add_argument("--model", default="small", help="模型大小: tiny/base/small/medium/large")
    parser.add_argument("--port", type=int, default=5555, help="监听端口")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址")
    args = parser.parse_args()

    # === 加载模型 (只执行一次，常驻显存) ===
    from faster_whisper import WhisperModel
    try:
        import torch
        if torch.cuda.is_available():
            device, compute_type = "cuda", "float16"
        else:
            device, compute_type = "cpu", "int8"
    except ImportError:
        device, compute_type = "cpu", "int8"

    print(f"[WhisperServer] 加载模型 '{args.model}' on {device} ...")
    model = WhisperModel(args.model, device=device, compute_type=compute_type)
    model_name = args.model
    print(f"[WhisperServer] ✓ 模型就绪！监听 {args.host}:{args.port}")
    print(f"[WhisperServer] 主程序可随意重启，本服务保持模型常驻。")
    print(f"[WhisperServer] 按 Ctrl+C 关闭服务。")

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
