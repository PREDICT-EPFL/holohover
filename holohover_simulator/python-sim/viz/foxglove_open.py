"""
Opens an MCAP file in the Foxglove web app. The app can't be pointed at a local file, so the file is
served over HTTP on this machine and the app is opened with a link to it (a "remote-file" data source):

    https://app.foxglove.dev/~/view?ds=remote-file&ds.url=http://127.0.0.1:<port>/<file>.mcap

The app reads the file with range requests and needs CORS headers on them. The server only listens on
127.0.0.1 and serves that one file until Ctrl-C. Chrome may ask once to let app.foxglove.dev access
apps on this device: allow it. In the devcontainer, the port must be forwarded to the host (VS Code's
Ports tab; it usually does it by itself).
"""

import http.server
import re
import sys
import webbrowser
from pathlib import Path
from urllib.parse import quote, urlencode, urlsplit

APP_URL = "https://app.foxglove.dev/~/view"
PORT = 8790  # the next free one if taken, e.g. by another viewer still running


def foxglove_link(file_url):
    """Link that opens the app on the MCAP file at `file_url`."""
    return f"{APP_URL}?{urlencode({'ds': 'remote-file', 'ds.url': file_url})}"


class FileHandler(http.server.BaseHTTPRequestHandler):
    """Serves `data` at `url_path`, whole or by byte range, to any origin."""

    url_path, data, etag = "/", b"", '""'

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Expose-Headers", "Accept-Ranges, Content-Length, Content-Range, ETag")
        super().end_headers()

    def do_OPTIONS(self):
        """CORS preflight: the Range header, and Chrome's check before a public site reaches localhost."""
        self.send_response(204)
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Range")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Access-Control-Max-Age", "86400")
        self.end_headers()

    def do_GET(self):
        self.send_file(body=True)

    def do_HEAD(self):
        self.send_file(body=False)

    def send_file(self, body):
        if urlsplit(self.path).path != self.url_path:
            self.send_error(404)
            return
        size = len(self.data)
        start, end = 0, size
        match = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", "").strip())
        if match and any(match.groups()):
            first, last = match.groups()
            if first:
                start, end = int(first), min(size, int(last) + 1) if last else size
            else:  # the last `last` bytes
                start = max(0, size - int(last))
            if start >= end:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self.send_response(206)
            self.send_header("Content-Range", f"bytes {start}-{end - 1}/{size}")
        else:
            self.send_response(200)
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(end - start))
        self.send_header("ETag", self.etag)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if body:
            self.wfile.write(self.data[start:end])

    def log_message(self, format, *args):
        pass


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # The app drops its first GET once it has the headers (file size, range support)
        if not isinstance(sys.exc_info()[1], ConnectionError):
            super().handle_error(request, client_address)


def serve(path, port=PORT):
    """A server for the file `path` on 127.0.0.1 (not started), and the file's URL."""
    path = Path(path)
    url_path = "/" + quote(path.name)
    stat = path.stat()
    handler = type("Handler", (FileHandler,), {"url_path": url_path, "data": path.read_bytes(),
                                               "etag": f'"{stat.st_size:x}-{stat.st_mtime_ns:x}"'})
    for candidate in range(port, port + 20):
        try:
            server = Server(("127.0.0.1", candidate), handler)
            break
        except OSError:
            continue
    else:
        server = Server(("127.0.0.1", 0), handler)
    return server, f"http://127.0.0.1:{server.server_port}{url_path}"


def open_in_foxglove(path, port=PORT, browser=True):
    """Serve `path` until Ctrl-C and open it in the Foxglove web app (the link is printed too)."""
    server, file_url = serve(path, port)
    link = foxglove_link(file_url)
    print(f"\nOpen in Foxglove: {link}\nServing {file_url} until Ctrl-C", flush=True)
    if browser:
        webbrowser.open(link)
    with server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print()
