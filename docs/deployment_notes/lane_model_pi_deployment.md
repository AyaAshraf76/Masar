# Deploying the Lane-Detection Model to the Raspberry Pi

Recovered and cleaned up from ad-hoc terminal notes on the old
`lane_bridge_node` GitHub branch (previously an unstructured pasted
file called `lost data`, which also contained a superseded early
draft of `perception/lane_detection/lane_stream_final.py` — that draft
was dropped here since the final script is already tracked in
`perception/lane_detection/`).

## 1. Copy the trained ONNX model to the Pi

From the laptop that has the trained model:

```bash
scp ~/Downloads/lane_model.onnx      pi@192.168.1.12:/home/pi/
scp ~/Downloads/lane_model.onnx.data pi@192.168.1.12:/home/pi/
```

On the Pi, confirm both files arrived:

```bash
ls -lh /home/pi/lane_model*
```

## 2. Install/upgrade the ONNX runtime on the Pi

```bash
pip3 install --upgrade onnxruntime numpy
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
```

## 3. Sanity-check the model loads and runs

```bash
python3 - << 'EOF'
import onnxruntime as ort
import numpy as np

model_path = "/home/pi/lane_model.onnx"

sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

print("Input name:", sess.get_inputs()[0].name)
print("Input shape:", sess.get_inputs()[0].shape)
print("Output shape:", sess.get_outputs()[0].shape)

dummy = np.random.rand(1, 3, 240, 320).astype(np.float32)
out = sess.run(None, {sess.get_inputs()[0].name: dummy})

print("Real output:", out[0].shape)
print("Model loaded successfully")
EOF
```

## 4. Benchmark inference speed on the Pi CPU

```bash
python3 - << 'EOF'
import onnxruntime as ort
import numpy as np
import time

model_path = "/home/pi/lane_model.onnx"

sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
input_name = sess.get_inputs()[0].name

dummy = np.random.rand(1, 3, 240, 320).astype(np.float32)

for _ in range(5):
    sess.run(None, {input_name: dummy})

times = []
for _ in range(30):
    t = time.perf_counter()
    sess.run(None, {input_name: dummy})
    times.append(time.perf_counter() - t)

print(f"Mean: {np.mean(times)*1000:.1f} ms")
print(f"P95 : {np.percentile(times, 95)*1000:.1f} ms")
print(f"FPS : {1/np.mean(times):.1f}")
EOF
```

## 5. Run the real pipeline

```bash
source ~/lane_env/bin/activate
python3 /home/pi/lane_stream_final.py
```

See `perception/lane_detection/README.md` for the full data flow
(this script → `lane_bridge_node` on the ROS2 laptop → downstream
consumers).
