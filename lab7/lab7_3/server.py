from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import requests

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        # Log request info to stdout
        print(f"Received GET request from {self.client_address}")
        print(f"Path: {self.path}")
        print(f"Headers:\n{self.headers}")

        r = requests.get("https://ipinfo.io/json").json()
        coord = r['loc']

        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        response = coord
        self.wfile.write(response.encode('utf-8'))

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)

        # Log request info to stdout
        print(f"Received POST request from {self.client_address}")
        print(f"Path: {self.path}")
        print(f"Headers:\n{self.headers}")
        print(f"Body:\n{post_data.decode('utf-8')}\n\n")

        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        response = {"message": "POST request received", "data": post_data.decode('utf-8')}
        self.wfile.write(json.dumps(response).encode('utf-8'))

def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=2234):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting server on port {port}...")
    httpd.serve_forever()

if __name__ == "__main__":
    run()