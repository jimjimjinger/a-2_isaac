from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

try:
    import uvicorn
except Exception:  # pragma: no cover - optional runtime dependency
    uvicorn = None

app = FastAPI()

rclpy.init()

node = rclpy.create_node("web_cmd_node")

pub = node.create_publisher(Twist, "/cmd_vel", 10)

STATIC_DIR = Path(__file__).resolve().parent / "static"


@app.get("/move")
def move(linear: float = 0.0, angular: float = 0.0):
    msg = Twist()

    msg.linear.x = linear
    msg.angular.z = angular

    pub.publish(msg)

    rclpy.spin_once(node, timeout_sec=0.0)

    return {"ok": True}


app.mount("/", StaticFiles(directory=STATIC_DIR, html=True))


if __name__ == "__main__":
    if uvicorn is None:
        raise RuntimeError("uvicorn is required to run web_controller directly")
    uvicorn.run(app, host="0.0.0.0", port=8001)
