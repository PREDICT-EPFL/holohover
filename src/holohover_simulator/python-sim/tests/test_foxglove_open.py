"""File server for the Foxglove web app: range requests and CORS headers, as the app's remote-file source needs."""

import threading
import urllib.error
import urllib.request
from urllib.parse import parse_qs, urlsplit

import pytest

from viz.foxglove_open import APP_URL, foxglove_link, serve

DATA = bytes(range(256)) * 4


@pytest.fixture(scope="module")
def file_url(tmp_path_factory):
    path = tmp_path_factory.mktemp("serve") / "run 1.mcap"
    path.write_bytes(DATA)
    server, url = serve(path, port=0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield url
    server.shutdown()
    server.server_close()


def request(url, method="GET", **headers):
    try:
        with urllib.request.urlopen(urllib.request.Request(url, method=method, headers=headers)) as response:
            return response.status, response.headers, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.headers, error.read()


def test_whole_file_announces_ranges(file_url):
    """The app's first GET: it reads the size and range support, then drops the connection."""
    status, headers, body = request(file_url, Origin="https://app.foxglove.dev")
    assert status == 200 and body == DATA
    assert headers["Accept-Ranges"] == "bytes" and headers["Content-Length"] == str(len(DATA))
    assert headers["Access-Control-Allow-Origin"] == "*"
    assert {"Accept-Ranges", "Content-Length", "ETag"} <= {
        h.strip() for h in headers["Access-Control-Expose-Headers"].split(",")}
    assert request(file_url, method="HEAD")[1]["Content-Length"] == str(len(DATA))


@pytest.mark.parametrize("range_, start, end", [("bytes=10-19", 10, 20), ("bytes=1000-", 1000, 1024),
                                                ("bytes=-24", 1000, 1024), ("bytes=1000-5000", 1000, 1024)])
def test_ranges(file_url, range_, start, end):
    status, headers, body = request(file_url, Range=range_)
    assert status == 206 and body == DATA[start:end]
    assert headers["Content-Range"] == f"bytes {start}-{end - 1}/{len(DATA)}"


def test_unsatisfiable_range_and_other_paths(file_url):
    status, headers, _ = request(file_url, Range="bytes=2000-")
    assert status == 416 and headers["Content-Range"] == f"bytes */{len(DATA)}"
    assert request(file_url.rsplit("/", 1)[0] + "/other.mcap")[0] == 404


def test_preflight(file_url):
    status, headers, _ = request(file_url, method="OPTIONS", Origin="https://app.foxglove.dev",
                                 **{"Access-Control-Request-Method": "GET", "Access-Control-Request-Headers": "range"})
    assert status == 204
    assert headers["Access-Control-Allow-Origin"] == "*" and headers["Access-Control-Allow-Headers"] == "Range"
    assert headers["Access-Control-Allow-Private-Network"] == "true"


def test_link(file_url):
    link = foxglove_link(file_url)
    assert link.startswith(APP_URL + "?")
    assert parse_qs(urlsplit(link).query) == {"ds": ["remote-file"], "ds.url": [file_url]}
    assert file_url.startswith("http://127.0.0.1:") and file_url.endswith("/run%201.mcap")
