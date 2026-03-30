#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve the local PuTTY live viewer and stream appended log lines.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    parser.add_argument("--port", type=int, default=8765, help="Bind port")
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[1]), help="Project root")
    parser.add_argument("--log", default="putty_log/putty.log", help="Default repo-relative log path")
    return parser.parse_args()


ARGS = parse_args()
ROOT_DIR = Path(ARGS.root).resolve()
DEFAULT_LOG_REL = ARGS.log.replace("\\", "/").lstrip("/")


def resolve_repo_path(relative_path: str) -> Path:
    normalized = (relative_path or DEFAULT_LOG_REL).replace("\\", "/").lstrip("/")
    candidate = (ROOT_DIR / normalized).resolve()
    try:
        candidate.relative_to(ROOT_DIR)
    except ValueError as exc:
        raise ValueError(f"path escapes project root: {relative_path}") from exc
    return candidate


class PuttyBridgeHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT_DIR), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        sys.stdout.write("[putty_live_bridge] " + (fmt % args) + "\n")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", f"/Doc/putty/index.html?autoconnect=bridge&path={DEFAULT_LOG_REL}")
            self.end_headers()
            return

        if parsed.path == "/api/status":
            self.handle_status(parsed)
            return

        if parsed.path == "/api/snapshot":
            self.handle_snapshot(parsed)
            return

        if parsed.path == "/api/live":
            self.handle_live(parsed)
            return

        super().do_GET()

    def handle_status(self, parsed) -> None:
        query = parse_qs(parsed.query)
        requested = query.get("path", [DEFAULT_LOG_REL])[0]
        path = resolve_repo_path(requested)
        payload = {
            "path": requested.replace("\\", "/"),
            "exists": path.exists(),
            "size": path.stat().st_size if path.exists() else 0,
            "mtime": path.stat().st_mtime if path.exists() else 0,
        }
        self.write_json(payload)

    def handle_snapshot(self, parsed) -> None:
        query = parse_qs(parsed.query)
        requested = query.get("path", [DEFAULT_LOG_REL])[0]
        path = resolve_repo_path(requested)

        text = ""
        size = 0
        mtime = 0
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="ignore")
            stat = path.stat()
            size = stat.st_size
            mtime = stat.st_mtime

        payload = {
            "path": requested.replace("\\", "/"),
            "exists": path.exists(),
            "size": size,
            "mtime": mtime,
            "text": text,
        }
        self.write_json(payload)

    def handle_live(self, parsed) -> None:
        query = parse_qs(parsed.query)
        requested = query.get("path", [DEFAULT_LOG_REL])[0]
        tail_requested = query.get("tail", ["0"])[0] == "1"
        path = resolve_repo_path(requested)

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        file_handle = None
        current_offset = 0
        start_from_end = tail_requested and path.exists()

        try:
            while True:
                if not path.exists():
                    if file_handle is not None:
                        try:
                            file_handle.close()
                        finally:
                            file_handle = None
                            current_offset = 0
                            self.emit_sse("reset", {"reason": "missing", "path": requested})
                    self.emit_keepalive()
                    time.sleep(0.25)
                    continue

                if file_handle is None:
                    file_handle = path.open("r", encoding="utf-8", errors="ignore")
                    if start_from_end:
                        file_handle.seek(0, os.SEEK_END)
                    current_offset = file_handle.tell()
                    self.emit_sse("ready", {"path": requested, "offset": current_offset})
                    start_from_end = False

                stat = path.stat()
                if stat.st_size < current_offset:
                    file_handle.close()
                    file_handle = path.open("r", encoding="utf-8", errors="ignore")
                    current_offset = 0
                    self.emit_sse("reset", {"reason": "truncated", "path": requested})

                line = file_handle.readline()
                if line:
                    current_offset = file_handle.tell()
                    self.emit_sse("line", {"path": requested, "offset": current_offset, "line": line.rstrip("\r\n")})
                    continue

                self.emit_keepalive()
                time.sleep(0.2)
        except (BrokenPipeError, ConnectionResetError):
            return
        finally:
            if file_handle is not None:
                file_handle.close()

    def emit_keepalive(self) -> None:
        self.wfile.write(b": keepalive\n\n")
        self.wfile.flush()

    def emit_sse(self, event_name: str, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False)
        message = f"event: {event_name}\ndata: {body}\n\n"
        self.wfile.write(message.encode("utf-8"))
        self.wfile.flush()

    def write_json(self, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main() -> None:
    print(f"[putty_live_bridge] root={ROOT_DIR}")
    print(f"[putty_live_bridge] default_log={DEFAULT_LOG_REL}")
    print(f"[putty_live_bridge] listening=http://{ARGS.host}:{ARGS.port}")
    server = ReusableThreadingHTTPServer((ARGS.host, ARGS.port), PuttyBridgeHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
