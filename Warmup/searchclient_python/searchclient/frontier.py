import heapq
from abc import ABC, abstractmethod
from collections import deque

from searchclient.heuristic import Heuristic
from searchclient.state import State


class Frontier(ABC):
    @abstractmethod
    def add(self, state: State) -> None: ...

    @abstractmethod
    def pop(self) -> State: ...

    @abstractmethod
    def is_empty(self) -> bool: ...

    @abstractmethod
    def size(self) -> int: ...

    @abstractmethod
    def contains(self, state: State) -> bool: ...

    @abstractmethod
    def get_name(self) -> str: ...


class FrontierBFS(Frontier):
    def __init__(self) -> None:
        super().__init__()
        self.queue: deque[State] = deque()
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        self.queue.append(state)
        self.set.add(state)

    def pop(self) -> State:
        state = self.queue.popleft()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.queue) == 0

    def size(self) -> int:
        return len(self.queue)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "breadth-first search"


class FrontierDFS(Frontier):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[State] = []
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        self.stack.append(state)
        self.set.add(state)

    def pop(self) -> State:
        state = self.stack.pop()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.stack) == 0

    def size(self) -> int:
        return len(self.stack)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "depth-first search"


class FrontierBestFirst(Frontier):
    """
    Priority-queue frontier for A*, WA*, and Greedy search.

    Uses a min-heap keyed by f(state) with a monotonic counter as a tie-breaker
    so Python never tries to compare two State objects directly.

    The set tracks exactly which states are currently live in the frontier, so
    size() and contains() are O(1) and is_empty() is correct even when stale
    heap entries exist (lazy-deletion pattern).
    """

    def __init__(self, heuristic: Heuristic) -> None:
        super().__init__()
        self.heuristic = heuristic
        self._heap: list[tuple[int, int, State]] = []
        self._set: set[State] = set()
        self._counter: int = 0

    def add(self, state: State) -> None:
        f = self.heuristic.f(state)
        heapq.heappush(self._heap, (f, self._counter, state))
        self._counter += 1
        self._set.add(state)

    def pop(self) -> State:
        # Skip stale heap entries (states already popped via lazy deletion).
        while self._heap:
            _, _, state = heapq.heappop(self._heap)
            if state in self._set:
                self._set.remove(state)
                return state
        raise IndexError("pop from empty frontier")

    def is_empty(self) -> bool:
        return len(self._set) == 0

    def size(self) -> int:
        return len(self._set)

    def contains(self, state: State) -> bool:
        return state in self._set

    def get_name(self) -> str:
        return f"best-first search using {self.heuristic}"