"""
HTTP proxy server that injects an Origin header into requests before forwarding.
SOLID design, single responsibility, uses built-in logging.
Designed to run as a separate process in the background.
"""

# ============================= Imports =============================== #
from http.server import BaseHTTPRequestHandler, HTTPServer
import requests
import logging
import sys


# =========================== Constants =============================== #
TARGET_URL = "http://localhost:8088"
ORIGIN_HEADER = "http://localhost:3000"
LISTEN_PORT = 6911
CHUNK_SIZE = 8192


# ========================== Logger Setup ============================= #
class ColoredFormatter(logging.Formatter):
    """
    Custom logging formatter to add colors based on log level.
    """
    COLORS = {
        logging.DEBUG: "\033[94m",   # Blue
        logging.INFO: "\033[92m",    # Green
        logging.WARNING: "\033[93m", # Yellow
        logging.ERROR: "\033[91m",   # Red
        logging.CRITICAL: "\033[95m" # Magenta
    }
    RESET = "\033[0m"

    def format(self, record):
        color = self.COLORS.get(record.levelno, self.RESET)
        record.msg = f"{color}{record.msg}{self.RESET}"
        return super().format(record)

logger = logging.getLogger("ProxyLogger")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler(sys.stdout)
ch.setFormatter(ColoredFormatter("[%(levelname)s] %(message)s"))
logger.addHandler(ch)


# ======================= Request Handler ============================ #
class ProxyRequestHandler(BaseHTTPRequestHandler):
    """
    HTTP request handler that forwards requests to a target server,
    injecting an Origin header, and logs requests and responses.
    """

    def do_GET(self) -> None:
        """
        Handle GET requests by forwarding them to the target server
        with the injected Origin header.
        """

        self._log_request()
        forward_headers = dict(self.headers)
        forward_headers["Origin"] = ORIGIN_HEADER

        target_url = f"{TARGET_URL}{self.path}"
        
        try:
            response = requests.get(target_url, headers=forward_headers, stream=True)
        
        except Exception as exception:
            logger.error(f"Failed to forward request: {exception}")
            self.send_error(502, f"Bad Gateway: {exception}")
            return

        self._log_response(response)
        self._send_response(response)


    def _log_request(self) -> None:
        """
        Log the incoming request details.
        """

        print("\nq_/========= Upcoming Request =========\_p\n")
        logger.info(f"Incoming Request: {self.command} {self.path}")
        
        for header, value in self.headers.items():
            logger.info(f"  {header}: {value}")


    def _log_response(self, response: requests.Response) -> None:
        """
        Log the response details from the target server.
        """

        print("\nq_/========= Response From Target Server =========\_p\n")
        logger.info(f"Response Status: {response.status_code} {response.reason}")
      
        for header, value in response.headers.items():
            logger.info(f"  {header}: {value}")


    def _send_response(self, response: requests.Response) -> None:
        """
        Send the response back to the client.
        """

        self.send_response(response.status_code)
        
        for header, value in response.headers.items():
            if header.lower() != "transfer-encoding":
                self.send_header(header, value)

        self.end_headers()
       
        for chunk in response.iter_content(chunk_size=CHUNK_SIZE):
            self.wfile.write(chunk)


    def log_message(self, format, *args):

        return  # suppress default logging


# ================================ Main ============================== #
def main():
    """
    Main function to start the proxy server.
    """
    
    server = HTTPServer(("", LISTEN_PORT), ProxyRequestHandler)
    logger.info(f"Proxy server running on port {LISTEN_PORT}, forwarding to {TARGET_URL} with Origin {ORIGIN_HEADER}")
    
    try:
        server.serve_forever()

    except KeyboardInterrupt:
        logger.info("Stopping proxy server...")
        server.shutdown()
        server.server_close()
        logger.info("Proxy server stopped.")


# ============================== Entry Point ========================= #
if __name__ == "__main__":
    main()
