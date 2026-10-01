"""Minimal browser UI for the commander (stdlib only: http.server + one static HTML page).

Endpoints (all served by the commander process):
  GET  /                  the page (share/llm_commander/web/index.html)
  GET  /api/state?since=N new log lines + status
  GET  /api/camera.jpg    live safety-camera frame
  GET  /api/judged.jpg    the frame the VLM judged last
  POST /api/command       {"text": "..."}  -> queued like a typed command
  POST /api/action        {"name": "..."}  -> scene / safety buttons
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from ament_index_python.packages import get_package_share_directory

BOOT_ID = str(time.time())  # lets an open page notice that the commander was restarted


def _index_path():
    return os.path.join(get_package_share_directory('llm_commander'), 'web', 'index.html')


def _actions(node):
    """name -> (queued?, function). Queued actions wait for the robot; the rest run immediately."""
    def safety(on):
        def f():
            if on and node.monitor is None:
                node.say('[error] No camera connected; restart the commander with --web or --safety.')
                return
            node.safety_enabled = on
            node.say(f"[safety] Safety check {'ON: every command starts with a camera check' if on else 'OFF'}.")
        return f

    def check_now():
        if node.monitor is None:
            node.say('[error] No camera connected.')
        else:
            node.check_safety()

    def home():
        node.say('[scene] Robot home...')
        if not node.dry_run:
            node.skills.home()
        node.say('[done] Robot is home.')

    return {
        'reset': (True, node.reset_scene),
        'home': (True, home),
        'spawn_person': (False, lambda: node.run_tool('spawn_person')),
        'remove_person': (False, lambda: node.run_tool('spawn_person', '--remove')),
        'safety_on': (False, safety(True)),
        'safety_off': (False, safety(False)),
        'check': (False, check_now),
    }


def _handler(node):
    actions = _actions(node)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # keep the terminal for robot output
            pass

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ('/', '/index.html'):
                with open(_index_path(), 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            if url.path == '/api/state':
                since = int(parse_qs(url.query).get('since', ['0'])[0])
                lines, seq = node.log_since(since)
                return self._send(200, {
                    'boot': BOOT_ID, 'lines': lines, 'seq': seq,
                    'busy': node.busy.is_set(), 'queued': node.commands.qsize(),
                    'model': node.planner.model, 'dry_run': node.dry_run,
                    'camera': node.monitor is not None, 'safety': node.safety_enabled,
                    'verdict': node.last_verdict and dict(  # age computed here: browser clocks may differ
                        node.last_verdict, age=round(time.time() - node.last_verdict['at'])),
                    'objects': node.live_objects(), 'destinations': list(node.dests),
                })
            if url.path == '/api/camera.jpg':
                jpg, _ = node.monitor.latest_jpeg(width=640) if node.monitor else (None, None)
                return self._send(200, jpg, 'image/jpeg') if jpg else self._send(503, {'error': 'no camera image'})
            if url.path == '/api/judged.jpg':
                jpg = node.monitor.last_judged_jpeg if node.monitor else None
                return self._send(200, jpg, 'image/jpeg') if jpg else self._send(404, {'error': 'no check yet'})
            return self._send(404, {'error': 'not found'})

        def do_POST(self):
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            except json.JSONDecodeError:
                return self._send(400, {'error': 'bad JSON'})
            if self.path == '/api/command':
                text = str(body.get('text', '')).strip()
                if not text:
                    return self._send(400, {'error': 'empty command'})
                node.commands.put(text)
                return self._send(200, {'queued': node.commands.qsize()})
            if self.path == '/api/action':
                name = body.get('name')
                if name not in actions:
                    return self._send(400, {'error': f'unknown action {name!r}'})
                queued, fn = actions[name]
                if queued:
                    node.commands.put(fn)
                else:
                    threading.Thread(target=fn, daemon=True).start()
                return self._send(200, {'ok': True})
            return self._send(404, {'error': 'not found'})

    return Handler


def start_web_ui(node, host='127.0.0.1', port=8080):
    server = ThreadingHTTPServer((host, port), _handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return f"http://{'localhost' if host in ('127.0.0.1', '0.0.0.0') else host}:{port}"
