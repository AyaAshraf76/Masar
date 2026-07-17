# Lane Detection

Camera-based lane detection running on the Raspberry Pi 5.

## Files

- `lane_model.onnx` + `lane_model_onnx.data` — trained segmentation model (3 classes: background, solid boundary, separator)
- `lane_stream_final.py` — runs on Pi, captures frames, runs inference, streams JSON data (port 9999) and annotated video (port 9998) to the laptop
- `lane_viewer.py` — runs on laptop, connects to the video stream and displays it (press Q to quit, F for fullscreen)
- `ros2_ws/src/lane_bridge/` — ROS2 node that receives the JSON stream and publishes lane topics

## Running

```bash
# On Pi:
python3 lane_stream_final.py

# On laptop (viewer):
python3 lane_viewer.py

# On laptop (ROS2 bridge):
ros2 run lane_bridge lane_bridge_node
```

The model takes 320×240 RGB input. Post-processing extracts lane center, width, which lane (inner/outer), error in cm, and curvature. Both lanes are 35cm wide.
