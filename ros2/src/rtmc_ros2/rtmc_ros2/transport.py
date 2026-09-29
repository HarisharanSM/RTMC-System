"""Bounded, proxy-free HTTP transport to the local simulation process."""
import json
import urllib.error
import urllib.parse
import urllib.request


class NoRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise urllib.error.HTTPError(req.full_url, code, 'redirects are forbidden', headers, fp)


class SimulatorClient:
    def __init__(self, base_url='http://127.0.0.1:8082'):
        parsed = urllib.parse.urlparse(base_url)
        if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost') or
                parsed.port != 8082 or parsed.username or parsed.password or
                parsed.path not in ('', '/') or parsed.query or parsed.fragment):
            raise ValueError('bridge accepts only http://127.0.0.1:8082 or http://localhost:8082')
        self.base_url = base_url.rstrip('/')
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirects())

    def _request(self, path, method='GET'):
        req = urllib.request.Request(self.base_url + path, method=method)
        with self.opener.open(req, timeout=.12) as reply:
            raw = reply.read(65537)
            if len(raw) > 65536:
                raise ValueError('simulator response exceeds 64 KiB')
            text = raw.decode('utf-8')
            value = json.loads(text)
            if not isinstance(value, dict):
                raise ValueError('expected simulator JSON object')
            return value, text

    def get_state(self):
        return self._request('/state')

    def dispatch(self, command, direction):
        query = urllib.parse.urlencode({'source': 'ros2',
                                        'btn': direction if command != 'stop' else 'none',
                                        'cmd': '' if command == 'hold' else command})
        return self._request('/command?' + query, 'POST')[0]
