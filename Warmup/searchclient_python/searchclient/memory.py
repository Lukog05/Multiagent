from math import inf

import psutil

max_usage = inf
_process = psutil.Process()

def get_usage() -> float:
    usage = _process.memory_info().rss / (1024 * 1024)
    assert isinstance(usage, float)
    return usage
