from math import inf

import psutil #psutil is a cross-platform library for retrieving information on running processes and system utilization (CPU, memory, disks, network, sensors) in Python.
#meaning that psutil can be used to monitor and manage system resources, including memory usage, which is relevant for the search client to ensure it does not exceed the specified maximum memory limit.

max_usage = inf
_process = psutil.Process()


def get_usage() -> float:
    """Returns memory usage of current process in MB."""
    usage = _process.memory_info().rss / (1024 * 1024)
    assert isinstance(usage, float)
    return usage
