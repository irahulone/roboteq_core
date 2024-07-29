import logging
import time
from collections import deque

log = logging.getLogger("queue_log")


class Queue:
    def __init__(self):
        self.request_queue = deque()
        self.response_queue = deque()
        self.retries = 3
        self.sleep_retry = 0.01  # seconds to sleep before retrying

    def get_response(self, key: str) -> dict:
        for retry in range(self.retries):
            for entry in self.response_queue:
                if entry.get(key):
                    return entry
            log.info(f"Response queue does not contain key: {key}, retrying")
            time.sleep(self.sleep_retry)

        return {}

    def set_response(self, req: str, resp: dict) -> None:
        if resp:
            self.response_queue.append(resp)
        else:
            log.info(f"Empty response from request {req}: {resp}")

    def get_request(self) -> dict:
        for retry in range(self.retries):
            try:
                return self.request_queue.popleft()
            except IndexError as e:
                log.info("Response queue is empty, retrying")
                time.sleep(self.sleep_retry)

        return {}

    def set_request(self, req):
        self.request_queue.append(req)

