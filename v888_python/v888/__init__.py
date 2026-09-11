from .drone import V888, V888Error, get_connected_ssid
from .video import VideoReceiver, VideoFrame

# Backward-compat alias (earlier version of this package used this name)
V888Drone = V888

__all__ = ["V888", "V888Drone", "V888Error", "VideoReceiver", "VideoFrame", "get_connected_ssid"]
