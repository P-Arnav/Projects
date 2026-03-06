from fastapi import APIRouter, WebSocket
from starlette.websockets import WebSocketDisconnect
from websocket.manager import manager

router = APIRouter()


@router.websocket("/ws")
async def websocket_endpoint(ws: WebSocket, client_type: str = "unknown"):
    await ws.accept()
    manager.connect(ws)
    await ws.send_json({
        "event": "WELCOME",
        "data": {
            "client_type": client_type,
            "connected_clients": manager.client_count,
        }
    })
    try:
        while True:
            # Keep connection alive; we only push from server
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)
