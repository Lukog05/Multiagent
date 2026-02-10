from abc import ABC, abstractmethod

from searchclient.state import State

#All abstract methods in a abstract class must be implemented in all the child classes. If a child class does not implement all the abstract methods, then it will also be an abstract class and cannot be instantiated.
class Heuristic(ABC):
    def __init__(self, initial_state: State) -> None:
        # Here's a chance to pre-process the static parts of the level.
        pass

    def h(self, state: State) -> int:
        raise NotImplementedError

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"

"""
Signature of h(state) method:

h(self, state: State) -> int
Breakdown:

Method name: h
Parameters:
    self - the heuristic object instance
    state: State - the state to evaluate
    Return type: int - estimated cost from state to goal
Current implementation: Raises NotImplementedError
"""

"""
HeuristicAstar class returns state.g  + self.h(state) in its f(state) method, which is the standard A* evaluation function combining the cost to reach the current state (g) and the heuristic estimate to reach the goal (h).
i.e., f(n) = g(n) + h(n)
g(n) and h(n) are exploration and exploitation respectively.

HeuristicGreedy class returns self.h(state) in its f(state) method, which is the standard greedy evaluation function that relies solely on the heuristic estimate to reach the goal, ignoring the cost to reach the current state.
i.e., f(n) = h(n)

HeuristicWeightedAStar class returns state.g + self.w * self.h(state) in its f(state) method, which is a weighted A* evaluation function that combines the cost to reach the current state (g) with a weighted heuristic estimate to reach the goal (w * h). The weight w allows for tuning the influence of the heuristic in the search process.
i.e., f(n) = g(n) + w * h(n)
"""

"""
All heuristics share:
    h(state) method (you implement the heuristic logic once)
    __init__ for preprocessing
But differ in:
    How f(state) combines g and h
    Search behavior and guarantees
"""

"""
__repr__ method is a dunder method.
It is used to define the string representation of an object, which is what you see when you print the object or inspect it in a debugger.
Ex:
Class Person(...):
    def __repr__(self):
        return f"Person(name={self.name}, age={self.age})"

person = Person()
without __repr__, printing person would show something like <__main__.Person object at 0x7f8b9c2d0>, which is not informative.
With __repr__, it would show Person(name=Alice, age=30), which is much more useful for debugging and understanding the object's state.
"""