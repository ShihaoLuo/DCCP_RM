import websocket
import json
import time
cmd = {"op":"publish",
        "id":"1",
        "topic":"/cmd_vel",
        "msg":{

               "linear": {"x":1.0,"y": 0.0,"z": 0.0},

               "angular": {"x": 0.0,"y": 0.0,"z": 0.0}

               }}
cmd = json.dumps(cmd)
print(cmd)
websocket.enableTrace(True)
ws = websocket.WebSocket()
ws.connect("ws://127.0.0.1:9090")
for i in range(10):
    ws.send(cmd)
    time.sleep(1)
ws.close()
