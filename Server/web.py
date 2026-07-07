#!/usr/bin/python3
"""
Web control interface for the Lafvin PiKart skid-steer robot.

Serves a mobile-friendly control page over HTTP with:
  * a WebSocket for commands (JSON or legacy CMD_#text) and live telemetry,
  * MJPEG streaming for the camera feed.

Telemetry (battery, odometry pose, wheel speeds, PID duties) is pushed to every
connected WebSocket client at ControlConfig.telemetry_hz.

Usage:
    sudo python3 web.py              # Web only (port 8080)
    sudo python3 web.py --with-tcp   # Web + legacy TCP server (5000/8000/8080)
"""
import asyncio
import os
import sys
import threading

from aiohttp import web

# Add Server directory to path so imports work.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from server import Server
from config import CONFIG
import protocol

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
WEB_PORT = CONFIG.network.web_port


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------
async def index_handler(request):
    return web.FileResponse(os.path.join(STATIC_DIR, 'index.html'))


def _wait_for_frame(output):
    """Blocking wait for the next camera frame (run in an executor)."""
    with output.condition:
        output.condition.wait(timeout=2.0)
        return output.frame


async def video_handler(request):
    """MJPEG stream over HTTP multipart."""
    srv = request.app['server']
    srv.start_camera()
    output = srv.streaming_output

    if output is None:
        return web.Response(status=503, text='Camera not available')

    response = web.StreamResponse()
    response.content_type = 'multipart/x-mixed-replace; boundary=frame'
    await response.prepare(request)

    loop = asyncio.get_event_loop()
    try:
        while True:
            frame = await loop.run_in_executor(None, _wait_for_frame, output)
            if frame is None:
                continue
            data = (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(frame)).encode() + b'\r\n'
                b'\r\n' + frame + b'\r\n'
            )
            await response.write(data)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        srv.release_camera()

    return response


async def command_handler(request):
    """POST endpoint: newline-separated commands, dispatched one by one."""
    srv = request.app['server']
    body = await request.text()
    count = 0
    for line in body.strip().split('\n'):
        line = line.strip()
        if line:
            srv.dispatch_command(line)
            count += 1
    return web.json_response({'dispatched': count})


async def websocket_handler(request):
    """WebSocket endpoint for commands (in) and telemetry (out)."""
    srv = request.app['server']
    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)
    request.app['ws_clients'].add(ws)
    print('WebSocket client connected')

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                for line in msg.data.strip().split('\n'):
                    line = line.strip()
                    if line:
                        srv.dispatch_command(line)
            elif msg.type == web.WSMsgType.ERROR:
                print(f'WebSocket error: {ws.exception()}')
    finally:
        request.app['ws_clients'].discard(ws)
        print('WebSocket client disconnected')

    return ws


async def status_handler(request):
    """Return current mode and battery voltage as JSON (HTTP poll fallback)."""
    srv = request.app['server']
    tel = srv.get_telemetry()
    return web.json_response({'mode': tel['mode'], 'battery': tel['battery']})


# ---------------------------------------------------------------------------
# Telemetry broadcast
# ---------------------------------------------------------------------------
async def telemetry_broadcaster(app):
    """Push telemetry JSON to all WebSocket clients at telemetry_hz."""
    srv = app['server']
    period = 1.0 / max(1.0, CONFIG.control.telemetry_hz)
    while True:
        await asyncio.sleep(period)
        clients = app['ws_clients']
        if not clients:
            continue
        tel = srv.get_telemetry()
        message = protocol.telemetry_message(
            battery=tel['battery'], mode=tel['mode'], drive=tel['drive'],
            extra={'signs': tel.get('signs')})
        for ws in list(clients):
            if ws.closed:
                clients.discard(ws)
                continue
            try:
                await ws.send_str(message)
            except (ConnectionResetError, RuntimeError):
                clients.discard(ws)


async def _start_background(app):
    app['telemetry_task'] = asyncio.create_task(telemetry_broadcaster(app))


async def _stop_background(app):
    task = app.get('telemetry_task')
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


# ---------------------------------------------------------------------------
# App wiring
# ---------------------------------------------------------------------------
def create_app(server_instance):
    app = web.Application()
    app['server'] = server_instance
    app['ws_clients'] = set()
    app.router.add_get('/', index_handler)
    app.router.add_get('/video', video_handler)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/status', status_handler)
    app.router.add_post('/command', command_handler)
    app.on_startup.append(_start_background)
    app.on_cleanup.append(_stop_background)
    return app


def run_web(server_instance, port=WEB_PORT):
    """Start the web server (blocking)."""
    app = create_app(server_instance)
    web.run_app(app, host='0.0.0.0', port=port,
                print=lambda *a: print(f'Web server running on port {port}'))


if __name__ == '__main__':
    srv = Server()

    if '--with-tcp' in sys.argv:
        srv.StartTcpServer()
        threading.Thread(target=srv.readdata,  daemon=True).start()
        threading.Thread(target=srv.sendvideo, daemon=True).start()
        threading.Thread(target=srv.Power,     daemon=True).start()
    try:
        run_web(srv)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        try:
            srv.drive.shutdown()
        except Exception:
            pass
        srv.stop_camera()
        srv.PWM.setMotorModel(0, 0, 0, 0)
