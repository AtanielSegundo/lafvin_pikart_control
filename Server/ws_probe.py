"""Throwaway minimal WebSocket client: read live telemetry from the kart."""
import base64, json, os, socket, struct, sys, time

HOST, PORT, PATH = "192.168.0.237", 8080, "/ws"


def recv_until(sock, marker):
    buf = b""
    while marker not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf


def read_frame(sock):
    hdr = sock.recv(2)
    if len(hdr) < 2:
        return None
    b1, b2 = hdr[0], hdr[1]
    opcode = b1 & 0x0F
    masked = b2 & 0x80
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack(">H", _recvn(sock, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", _recvn(sock, 8))[0]
    mask = _recvn(sock, 4) if masked else b""
    payload = _recvn(sock, length)
    if masked:
        payload = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
    return opcode, payload


def _recvn(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            break
        buf += chunk
    return buf


def main():
    s = socket.create_connection((HOST, PORT), timeout=8)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (
        f"GET {PATH} HTTP/1.1\r\nHost: {HOST}:{PORT}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(req.encode())
    resp = recv_until(s, b"\r\n\r\n")
    if b"101" not in resp.split(b"\r\n")[0]:
        print("handshake failed:", resp[:120]); return
    print("connected, reading telemetry...")

    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1000.0
    deadline = time.time() + dur
    last = None
    s.settimeout(6)
    while time.time() < deadline:
        frame = read_frame(s)
        if frame is None:
            break
        opcode, payload = frame
        if opcode != 0x1:
            continue
        try:
            msg = json.loads(payload.decode())
        except Exception:
            continue
        if msg.get("type") != "telemetry":
            continue
        d = msg.get("drive", {})
        p = d.get("pose", {})
        enc = d.get("encoders", {})
        last = (p, enc, d.get("goal_active"), d.get("duty"))
        print(f"x={p.get('x'):+.3f} y={p.get('y'):+.3f} "
              f"theta={p.get('theta_deg'):+.2f}deg  "
              f"raw_counts={enc}  duty={d.get('duty')}")
    s.close()
    if last:
        p = last[0]
        print(f"\nLATEST -> x={p.get('x')} m  y={p.get('y')} m  "
              f"theta={p.get('theta_deg')} deg  goal_active={last[1]}")


if __name__ == "__main__":
    main()
