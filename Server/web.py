#!/usr/bin/python3
"""
Web control interface for the Lafvin PiKart skid-steer robot.

Serves a mobile-friendly control page over HTTP with:
  * a WebSocket for commands (JSON or legacy CMD_#text) and live telemetry,
  * MJPEG streaming for the camera feed.

This is the parent process (P_web) of the multiprocess stack: it forks the
control, sensor, camera and aux processes (via Server -> Supervisor) and then
does nothing but network I/O. Telemetry is pushed to every connected WebSocket
client at ControlConfig.telemetry_hz -- now 20 Hz rather than 500, because the
control loop only produces a new snapshot at loop_hz and re-serialising the same
one 25 times over was costing a core the control loop needed.

Usage:
    sudo python3 web.py              # Web only (port 8080)
    sudo python3 web.py --with-tcp   # Web + legacy TCP server (5000/8000/8080)
    sudo python3 web.py --no-camera  # Skip the camera process
"""
import asyncio
import concurrent.futures
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

# Threads reserved for MJPEG viewers. Each in-flight viewer holds one for the
# duration of a blocking frame read, so this is also the cap on simultaneous
# streams -- beyond it, extra viewers queue for a slot instead of crowding out
# the rest of the process.
VIDEO_POOL_WORKERS = 4


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------
async def index_handler(request):
    return web.FileResponse(os.path.join(STATIC_DIR, 'index.html'))


async def video_handler(request):
    """MJPEG stream over HTTP multipart.

    Frames come out of the shared ring that P_camera writes, so this handler no
    longer shares a process with the JPEG encoder. The blocking ring read runs in
    an executor thread; each client has its own cursor, so a slow viewer skips
    frames rather than holding up the camera or the other viewers.
    """
    srv = request.app['server']
    reader = srv.acquire_camera()

    response = web.StreamResponse()
    response.content_type = 'multipart/x-mixed-replace; boundary=frame'
    await response.prepare(request)

    loop = asyncio.get_event_loop()
    try:
        while True:
            frame = await loop.run_in_executor(request.app['video_pool'],
                                               reader.read, 2.0)
            if frame is None:
                # Timed out: either the camera has not started yet or it stopped.
                # Fall through so a disconnected client is noticed on the next
                # write instead of blocking here forever.
                if request.transport is None or request.transport.is_closing():
                    break
                continue
            # Written in parts rather than one concatenated buffer: the old code
            # built `header + frame + trailer` per frame per viewer, copying the
            # whole JPEG on the event-loop thread.
            await response.write(
                b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                + str(len(frame)).encode() + b'\r\n\r\n')
            await response.write(frame)
            await response.write(b'\r\n')
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        reader.close()

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


async def health_handler(request):
    """Per-process health: pids, liveness, control-loop heartbeat age.

    New endpoint, and the thing to check first when the kart misbehaves: with the
    stack split across processes, "the web UI is up" no longer implies the control
    loop is running.
    """
    srv = request.app['server']
    return web.json_response(srv.supervisor.status())


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
            extra={'signs': tel.get('signs'), 'servo': tel.get('servo'),
                   'gains': tel.get('gains'),
                   'light': tel.get('light'), 'line': tel.get('line'),
                   'standstill': tel.get('standstill'),
                   'processes': tel.get('processes')})
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


async def _shutdown_video_pool(app):
    pool = app.get('video_pool')
    if pool is not None:
        # Don't wait: the workers may be mid-read with up to 2 s left on the
        # clock, and shutdown should not block on a viewer that already left.
        pool.shutdown(wait=False, cancel_futures=True)


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
    # A POOL OF ITS OWN, not asyncio's default executor.
    #
    # Each viewer parks a thread in a blocking ring read for up to 2 s at a
    # time, and re-enters it immediately. On the default executor -- shared with
    # everything else this process ever offloads -- two viewers were enough to
    # keep it permanently saturated with tasks that are blocked in an OS
    # semaphore (which, unlike a sleep, yields nothing cooperatively). Giving
    # video its own bounded pool means a crowd of viewers can starve video and
    # nothing else.
    app['video_pool'] = concurrent.futures.ThreadPoolExecutor(
        max_workers=VIDEO_POOL_WORKERS, thread_name_prefix='video')
    app.on_cleanup.append(_shutdown_video_pool)
    app.router.add_get('/', index_handler)
    app.router.add_get('/video', video_handler)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/status', status_handler)
    app.router.add_get('/health', health_handler)
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
    # Server() forks the child processes. It must therefore run before anything
    # here starts a thread or an event loop -- see Server.__init__ and
    # supervisor.assert_fork_safe.
    srv = Server(with_camera='--no-camera' not in sys.argv)

    # The parent keeps off the core reserved for the control loop.
    import ipc as ipc_mod
    ipc_mod.apply_process_tuning("web", cpu=CONFIG.process.web_cpus)

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
        # One call now: it releases the camera, stops the control loop (which
        # brakes the motors), joins every child and stops the motors again as a
        # backstop if any of that failed.
        srv.shutdown()
