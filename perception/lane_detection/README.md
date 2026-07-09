# Lane Detection

Camera-based lane detection running on the Raspberry Pi 5.

## Files

- `lane_model.onnx` + `lane_model_onnx.data` — trained segmentation model (3 classes: background, solid boundary, separator)
- `lane_stream_final.py` — runs on Pi, captures frames, runs inference, streams JSON data (port 9999) and annotated video (port 9998) to the laptop
- `lane_viewer.py` — runs on laptop, connects to the video stream and displays it (press Q to quit, F for fullscreen)
- `ros2_ws/src/lane_bridge/` — ROS2 node that receives the JSON stream and publishes lane topics
- `Lane_Detection_Lane_Keeping_Chapter__3_.docx` — thesis chapter on lane detection and lane keeping
- `Lane_Detection_Challenges_Report.docx` — challenges faced during development

The model takes 320×240 RGB input. Post-processing extracts lane center, width, which lane (inner/outer), error in cm, and curvature. Both lanes are 35cm wide.

## Deploying the model to the Pi

Copy the model files:
```bash
scp lane_model.onnx pi@192.168.1.12:/home/pi/
scp lane_model_onnx.data pi@192.168.1.12:/home/pi/
```

Install ONNX runtime on the Pi:
```bash
pip3 install --upgrade onnxruntime numpy
```

Quick sanity check that it loads:
```bash
python3 -c "
import onnxruntime as ort, numpy as np
sess = ort.InferenceSession('/home/pi/lane_model.onnx', providers=['CPUExecutionProvider'])
dummy = np.random.rand(1, 3, 240, 320).astype(np.float32)
out = sess.run(None, {sess.get_inputs()[0].name: dummy})
print('Output shape:', out[0].shape, '— model loaded OK')
"
```

Benchmark inference speed:
```bash
python3 -c "
import onnxruntime as ort, numpy as np, time
sess = ort.InferenceSession('/home/pi/lane_model.onnx', providers=['CPUExecutionProvider'])
inp = sess.get_inputs()[0].name
d = np.random.rand(1,3,240,320).astype(np.float32)
[sess.run(None,{inp:d}) for _ in range(5)]  # warmup
t = [time.perf_counter() for _ in range(31)]
for i in range(30): sess.run(None,{inp:d}); t[i+1] = time.perf_counter()
dt = [t[i+1]-t[i] for i in range(30)]
print(f'Mean: {np.mean(dt)*1000:.1f} ms, FPS: {1/np.mean(dt):.1f}')
"
```

## Running

```bash
# On Pi:
source ~/lane_env/bin/activate
python3 lane_stream_final.py

# On laptop (viewer):
python3 lane_viewer.py

# On laptop (ROS2 bridge):
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run lane_bridge lane_bridge_node
```
