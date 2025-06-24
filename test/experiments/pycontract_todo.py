

##########################################
# TODO: TRANSFER FROM ORSTATE EXPERIMEnt #
##########################################

from __future__ import annotations

import inspect

"""
This is a version of PyContract (https://github.com/pyrv/pycontract), named MyContract (Mini PyContract)
that has been re-developed from scratch to support alternating automata. It is an experiment
not including all features of PyContract. A merge of the two is planned.
"""

from dataclasses import dataclass
from typing import Any, List, Optional, Union, Callable

Event = Any
Trace = List[Event]

# ----------------------------------------------------------------------------
# Auxiliary functions
# ----------------------------------------------------------------------------

COLOR_RESET = "\033[0m"
COLOR_YELLOW = "\033[93m"       # bright yellow
COLOR_GREEN = "\033[92m"        # green
COLOR_RED = "\033[91m"          # red
COLOR_ORANGE = "\033[38;5;208m" # vivid orange
COLOR_CYAN = "\033[96m"         # cyan

def color(col: str, msg: str) -> str:
    return f"{col}{msg}{COLOR_RESET}"

def yellow(msg: str):
    return color(COLOR_YELLOW, msg)

def green(msg: str):
    return color(COLOR_GREEN, msg)

def red(msg: str):
    return color(COLOR_RED, msg)

def orange(msg: str):
    return color(COLOR_ORANGE, msg)

def cyan(msg: str):
    return color(COLOR_CYAN, msg)

def pretty(expr: StateExpr, indent: int = 0) -> str:
    pad = '  ' * indent
    if isinstance(expr, (AndState, OrState, Sequence, UnorderedState)):
        lines = [f"{pad}{expr.__class__.__name__}:"]
        for s in expr.substates():
            lines.append(pretty(s, indent + 1))
        return "\n".join(lines)
    elif isinstance(expr, HotState):
        return f"{pad}{COLOR_YELLOW}{expr}{COLOR_RESET}"
    elif isinstance(expr, StateObject):
        return f"{pad}{COLOR_GREEN}{expr}{COLOR_RESET}"
    else:
        return f"{pad}{expr}"

def is_transition_method(member: object) -> bool:
    return inspect.ismethod(member) and member.__name__ == 'transition'

# ----------------------------------------------------------------------------
# States
# ----------------------------------------------------------------------------

class StateExpr:
    monitor: Optional['Monitor'] = None

    def __str__(self):
        fields = [f"{k}={v}" for k, v in self.__dict__.items() if k != 'monitor']
        return f"{self.__class__.__name__}({', '.join(fields)})"

    def eval(self, event: Event) -> Optional['StateExpr']:
        raise NotImplementedError

    def substates(self) -> List['StateExpr']:
        return []

    def __rshift__(self, other) -> Sequence:
        return Sequence(self, other)

    def report_safety_error(self, msg: str):
        self.monitor.report_safety_error(msg)

    def safety_error(self, msg: str) -> Error:
        self.monitor.report_safety_error(msg)
        return error

    def ensure(self, b: bool, msg='Assertion violation') -> Ok | Error:
        if b:
            return ok
        else:
            return self.safety_error(msg)

    def ok_message(self, msg: str) -> Ok:
        self.monitor.report_ok(msg)
        return ok

class Ok(StateExpr):
    def __repr__(self):
        return "ok"

class Error(StateExpr):
    def __repr__(self):
        return "error"

ok = Ok()
error = Error()

class StateObject(StateExpr):
    def eval(self, event: Event) -> StateExpr:
        result = self.transition(event)
        if result is None:
            return self
        else:
            return Monitor.inject_monitor(result, self.monitor)

    def transition(self, event: Event) -> Optional[StateExpr]:
        raise NotImplementedError

class HotState(StateObject):
    pass

class AlwaysState(StateObject):
    def eval(self, event: Event) -> StateExpr:
        result = self.transition(event)
        if result is None or result == ok or result == error:
            return self
        else:
            return Monitor.inject_monitor(AndState([self, result]), self.monitor)

# DONE
class OrState(StateExpr):
    def __init__(self, *options: Union[StateExpr, List[StateExpr]]):
        if len(options) == 1 and isinstance(options[0], list):
            self.options = options[0]
        else:
            self.options = list(options)

    def __str__(self):
        inner = ", ".join(str(s) for s in self.options)
        return f"Or([{inner}])"

    def substates(self) -> List['StateExpr']:
        return self.options

    def eval(self, event: Event) -> StateExpr:
        next_options = []
        for s in self.options:
            result = s.eval(event)
            if result == ok:
                return ok
            elif result == error:
                continue
            elif isinstance(result, OrState):
                next_options.extend(result.options)
            elif result:
                next_options.append(result)
            else:
                next_options.append(s)
        if not next_options:
            return error
        elif len(next_options) == 1:
            return next_options[0]
        else:
            return OrState(next_options)

# DONE
class AndState(StateExpr):
    def __init__(self, *options: Union[StateExpr, List[StateExpr]]):
        if len(options) == 1 and isinstance(options[0], list):
            self.options = options[0]
        else:
            self.options = list(options)

    def __str__(self):
        inner = ", ".join(str(s) for s in self.options)
        return f"And([{inner}])"

    def substates(self) -> List['StateExpr']:
        return self.options

    def eval(self, event: Event) -> Optional[StateExpr]:
        next_options = []
        for s in self.options:
            result = s.eval(event)
            if result == error or result == ok:
                continue
            elif isinstance(result, AndState):
                next_options.extend(result.options)
            elif result:
                next_options.append(result)
            else:
                next_options.append(s)
        if not next_options:
            return ok
        elif len(next_options) == 1:
            return next_options[0]
        else:
            return AndState(next_options)

# DONE
class NotState(StateExpr):
    def __init__(self, inner: StateExpr):
        self.inner = inner
        self.monitor: Optional['Monitor'] = None

    def __str__(self):
        return f"Not({self.inner})"

    def substates(self) -> List[StateExpr]:
        return [self.inner]

    def eval(self, event: Event) -> StateExpr:
        result = self.inner.eval(event)
        if result == ok:
            return error
        elif result == error:
            return ok
        else:
            return NotState(result)

class Sequence(StateExpr):
    def __init__(self, first: StateExpr, second: StateExpr):
        self.first = first
        self.second = second

    def __str__(self):
        return f"({self.first} >> {self.second})"

    def substates(self) -> List[StateExpr]:
        return [self.first, self.second]

    def eval(self, event: Event) -> StateExpr:
        result = self.first.eval(event)
        if result == ok:
            return Monitor.inject_monitor(self.second, self.monitor)
        elif result == error:
            return error
        else:
            return Monitor.inject_monitor(Sequence(result, self.second), self.monitor)

class UnorderedState(StateExpr):
    def __init__(self, *goals: StateExpr):
        self.remaining = list(goals)

    def __str__(self):
        inner = ", ".join(str(g) for g in self.remaining)
        return f"Unordered([{inner}])"

    def substates(self) -> List['StateExpr']:
        return self.remaining

    def eval(self, event: Event) -> StateExpr:
        next_remaining = []
        for s in self.remaining:
            result = s.eval(event)
            if result == ok:
                continue
            elif result == error:
                return error
            else:
                next_remaining.append(result)
        if not next_remaining:
            return ok
        else:
            return Monitor.inject_monitor(UnorderedState(*next_remaining), self.monitor)

# ----------------------------------------------------------------------------
# Combinators
# ----------------------------------------------------------------------------

def join(*goals: StateExpr, then: StateExpr) -> StateExpr:
    return UnorderedState(*goals) >> then

StateExpr.__or__ = lambda self, other: OrState(self, other)
StateExpr.__and__ = lambda self, other: AndState(self, other)

# ----------------------------------------------------------------------------
# Monitor
# ----------------------------------------------------------------------------

class Monitor:
    # Options:
    DEBUG_INFO: bool = False
    DEBUG_INTO_AFTER: bool = True

    debug_info: list[str] = []

    def __init__(self):
        super().__init__()
        self.is_top_monitor: bool = True
        self.monitors: List[Monitor] = []
        self.active: StateExpr = None
        self.safety_errors: List[str] = []
        self.liveness_errors: List[str] = []
        self.ok_messages: List[str] = []
        outer_transitions = inspect.getmembers(self, predicate=is_transition_method)
        if len(outer_transitions) > 0:
            (name, method) = outer_transitions[0]
            always = type("Always", (AlwaysState,), {})
            setattr(always, name, method.__func__)
            setattr(self, "Always", always)
            self.initial(always())

    def initial(self, *initial_states: StateObject):
        if len(initial_states) > 1:
            state = AndState(*initial_states)
        elif len(initial_states) == 1:
            state = initial_states[0]
        else:
            state = ok
        self.active = Monitor.inject_monitor(state, self)

    def monitor(self, *monitors: Monitor):
        for monitor in monitors:
            monitor.is_top_monitor = False
            self.monitors.append(monitor)

    def get_monitor_name(self) -> str:
        return self.__class__.__name__

    @staticmethod
    def debug(s: str = ''):
        if Monitor.DEBUG_INFO:
            print(s)
            Monitor.debug_info.append(s)

    @staticmethod
    def inject_monitor(expr: 'StateExpr', monitor: 'Monitor') -> 'StateExpr':
        if isinstance(expr, StateObject):
            expr.monitor = monitor
        elif isinstance(expr, (AndState, OrState, UnorderedState, Sequence)):
            expr.monitor = monitor
            for s in expr.substates():
                Monitor.inject_monitor(s, monitor)
        return expr

    def report_ok(self, msg: str):
        formatted = f"{COLOR_GREEN}!!! ok: {msg}{COLOR_RESET}"
        print(formatted)
        self.ok_messages.append(formatted)
        if Monitor.DEBUG_INFO:
            Monitor.debug_info.append(formatted)

    def report_safety_error(self, msg: str):
        formatted = f"{COLOR_RED}*** safety error: {msg}{COLOR_RESET}"
        print(formatted)
        self.safety_errors.append(formatted)
        Monitor.debug_info.append(formatted)

    def report_liveness_error(self, s: HotState):
        formatted = f"{COLOR_ORANGE}*** liveness error: {s}{COLOR_RESET}"
        print(formatted)
        self.liveness_errors.append(formatted)

    def exists(self, cls: Union[type, str], **fields) -> bool:
        def match_class(expr):
            if isinstance(cls, str):
                return expr.__class__.__name__ == cls
            else:
                return isinstance(expr, cls)

        def search(expr: StateExpr) -> bool:
            if match_class(expr) and all(getattr(expr, k, None) == v for k, v in fields.items()):
                return True
            elif isinstance(expr, (AndState, OrState, Sequence, UnorderedState)):
                return any(search(sub) for sub in expr.substates())
            return False

        return search(self.active)

    def print_event(self, e):
        Monitor.debug()
        event_info = f'@ {e}'
        line = '-' * len(event_info)
        Monitor.debug(line)
        Monitor.debug(event_info)
        Monitor.debug(line)

    def print_monitor_name(self, name):
        Monitor.debug(f'\nMonitor: {name}')

    def print_state(self):
        Monitor.debug(pretty(self.active))

    def eval(self, e: Event):
        if self.is_top_monitor:
            self.print_event(e)
        for monitor in self.monitors:
            self.print_monitor_name(monitor.get_monitor_name())
            monitor.eval(e)
        if self.active and self.active != ok and self.active != error:
            self.active = self.active.eval(e)
            self.print_state()

    def end(self):
        if self.is_top_monitor:
            print()
            print(color(COLOR_CYAN, '======================'))
            print(color(COLOR_CYAN, 'TERMINATING MONITORING'))
            print(color(COLOR_CYAN, '======================'))
            print()
            if Monitor.DEBUG_INTO_AFTER and Monitor.debug_info:
                print(color(COLOR_CYAN, 'DEBUG INFO:'))
                for info in Monitor.debug_info:
                    print(info)
                print()
            print(color(COLOR_CYAN, 'REPORT:'))

        for monitor in self.monitors:
            monitor.end()

        def collect_hot(expr: StateExpr, negated: bool = False) -> List[HotState]:
            if isinstance(expr, HotState):
                return [] if negated else [expr]
            elif isinstance(expr, NotState):
                return collect_hot(expr.inner, not negated)
            elif isinstance(expr, (AndState, OrState, Sequence, UnorderedState)):
                result = []
                for s in expr.substates():
                    result.extend(collect_hot(s, negated))
                return result
            else:
                return []

        line = '-' * len(f'Monitor {self.get_monitor_name()}:')
        print()
        print(line)
        print(f'Monitor {self.get_monitor_name()}:')
        print(line)

        print('\n--- SAFETY ERROR REPORT: ---\n')
        if self.safety_errors:
            for err in self.safety_errors:
                print(err)
        else:
            print('No safety errors!')

        print('\n--- LIVENESS ERROR REPORT: ---\n')
        hot_states = collect_hot(self.active)
        if hot_states:
            for s in collect_hot(self.active):
                self.report_liveness_error(s)
        else:
            print('No liveness property violations!')

        print('\n--- OK REPORT: ---\n')
        if self.ok_messages:
            for ok_msg in self.ok_messages:
                print(ok_msg)
        else:
            print('No ok messages!')

# ----------------------------------------------------------------------------
# Specification (Test)
# ----------------------------------------------------------------------------

@dataclass
class Acquire:
    t: str
    x: int

@dataclass
class Release:
    t: str
    x: int

@dataclass
class Cancel:
    t: str
    x: int

@dataclass
class Report:
    t: str
    x: int

@dataclass
class Write:
    t: str
    v: str


class MyMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.initial(self.Initial())

    @dataclass
    class Initial(AlwaysState):
        def transition(self, event: Event):
            match event:
                case Acquire(t, x):
                    return MyMonitor.Locked(t, x)
                case Release(t, x):
                    if not self.monitor.exists(MyMonitor.Locked, t=t, x=x):
                        return self.safety_error(f"lock {x} released by {t} without being held")

    @dataclass
    class Locked(HotState):
        t: str
        x: int

        def transition(self, event: Event):
            match event:
                case Acquire(t2, self.x) if t2 != self.t:
                    self.report_safety_error(f'Lock {self.x} acquired by {self.t} is acquired by {t2}')
                    return OrState(
                            MyMonitor.DoRelease(self.t, self.x),
                            MyMonitor.DoReport(self.t, self.x)
                    )
                case Release(self.t, self.x):
                    return ok

    @dataclass
    class DoRelease(HotState):
        t: str
        x: int

        def transition(self, event: Event):
            match event:
                case Release(self.t, self.x):
                    return ok

    @dataclass
    class DoReport(HotState):
        t: str
        x: int

        def transition(self, event: Event):
            match event:
                case Report(self.t, self.x):
                    return MyMonitor.DoCancel(self.t, self.x)

    @dataclass
    class DoCancel(HotState):
        t: str
        x: int

        def transition(self, event: Event):
            match event:
                case Cancel(self.t, self.x):
                    return ok


class MySuccinctMonitor(Monitor):
    def __init__(self):
        super().__init__()
        class Initial(AlwaysState):
            def transition(self, e):
                match e:
                    case Acquire(t, x):
                        class WaitForRelease(HotState):
                            def transition(self, e2):
                                match e2:
                                    case Acquire(_, y) if y == x:
                                        return self.safety_error(f'Acquire({t},{x}) followed by Acquire(_,{x})')
                                    case Release(u, y) if u == t and y == x:
                                        return ok
                        return WaitForRelease()
        self.initial(Initial())


class MySuccinctComplexMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.initial(self.Initial())

    class Initial(AlwaysState):
        def transition(self, event: Event):
            match event:
                case Acquire(t, x):
                    class Locked(HotState):
                        def transition(self, event):
                            match event:
                                case Acquire(t2, x2) if x2 == x and t2 != t:
                                    self.report_safety_error(f'Lock {x} acquired by {t} is acquired by {t2}')
                                    class DoRelease(HotState):
                                        def transition(self, event):
                                            match event:
                                                case Release(t2, x2) if t2 == t and x2 == x:
                                                    return ok

                                    class DoReport(HotState):
                                        def transition(self, event):
                                            match event:
                                                case Report(t2, x2) if t2 == t and x2 == x:
                                                    class DoCancel(HotState):
                                                        def transition(self, event):
                                                            match event:
                                                                case Cancel(t2, x2) if t2 == t and x2 == x:
                                                                    return ok
                                                    return DoCancel()
                                    return OrState(DoRelease(), DoReport())
                                case Release(t2, x2) if t2 == t and x2 == x:
                                    return ok
                    return Locked()
                case Release(t, x):
                    if not self.monitor.exists('Locked', t=t, x=x):
                        return self.safety_error(f"lock {x} released by {t} without being held")

class MySuccinctComplexBindingMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.initial(self.Initial())

    class Initial(AlwaysState):
        def transition(self, event: Event):
            match event:
                case Acquire(t, x):
                    class Locked(HotState):
                        def __init__(self, t, x):
                            self.t = t
                            self.x = x

                        def transition(self, event):
                            match event:
                                case Acquire(t2, x2) if x2 == self.x and t2 != self.t:
                                    self.report_safety_error(f'Lock {self.x} acquired by {self.t} is acquired by {t2}')

                                    class DoRelease(HotState):
                                        def __init__(self, t, x):
                                            self.t = t
                                            self.x = x

                                        def transition(self, event):
                                            match event:
                                                case Release(t2, x2) if t2 == self.t and x2 == self.x:
                                                    return ok

                                    class DoReport(HotState):
                                        def __init__(self, t, x):
                                            self.t = t
                                            self.x = x

                                        def transition(self, event):
                                            match event:
                                                case Report(t2, x2) if t2 == self.t and x2 == self.x:

                                                    class DoCancel(HotState):
                                                        def __init__(self, t, x):
                                                            self.t = t
                                                            self.x = x

                                                        def transition(self, event):
                                                            match event:
                                                                case Cancel(t2, x2) if t2 == self.t and x2 == self.x:
                                                                    return ok
                                                    return DoCancel(self.t, self.x)

                                    return DoRelease(self.t, self.x) | DoReport(self.t, self.x)

                                case Release(t2, x2) if t2 == self.t and x2 == self.x:
                                    return ok

                    return Locked(t, x)

                case Release(t, x):
                    if not self.monitor.exists('Locked', t=t, x=x):
                        return self.safety_error(f"lock {x} released by {t} without being held")

class MyUnorderedMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.initial(self.Initial())

    class Initial(AlwaysState):
        def transition(self, event: Event):
            match event:
                case Acquire(t, x):
                    class ReportExpected(HotState):
                        def transition(self, event):
                            match event:
                                case Report(t2, x2) if t2 == t and x2 == x:
                                    return self.ok_message(f'saw report after acquire')

                    class WriteExpected(HotState):
                        def transition(self, event):
                            match event:
                                case Write(t2, _) if t2 == t:
                                    return ok

                    class ReleaseExpected(HotState):
                        def transition(self, event):
                            match event:
                                case Release(t2, x2) if t2 == t and x2 == x:
                                    return ok

                    return UnorderedState(ReportExpected(), WriteExpected()) >> ReleaseExpected()
                # case Release(t, x):
                #     if not self.monitor.exists("Locked", t=t, x=x):
                #         return safety_error(f"lock {x} released by {t} without being held")

class NeverAcquireMonitor(Monitor):
    def __init__(self):
        super().__init__()
        class EventuallyAcquire(HotState):
            def transition(self, e):
                match e:
                    case Acquire(t, x):
                        return self.safety_error(f'Acquire({t},{x}) observed')
        self.initial(EventuallyAcquire())

class AllMonitors(Monitor):
    def __init__(self):
        super().__init__()
        self.monitor(
            MyMonitor(),
            MySuccinctMonitor(),
            MySuccinctComplexMonitor(),
            MySuccinctComplexBindingMonitor(),
            MyUnorderedMonitor(),
            NeverAcquireMonitor()
        )

class MyNegationMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.initial(
            NotState(MyNegationMonitor.EventuallySomething()),
            # MyNegationMonitor.EventuallySomething()
        )

    class EventuallySomething(HotState):
        def transition(self, event: Event):
            match event:
                case Report(t, x):
                    return ok


# ----------------------------------------------------------------------------
# Main program
# ----------------------------------------------------------------------------

if __name__ == '__main__':
    trace1= [Acquire("t1", 1)]
    trace2 = [Acquire("t1", 1), Acquire("t2", 1), Release("t2", 1), Report("t1", 1), Cancel("t1", 1)]
    trace3 = [Acquire("t1", 1), Release("t1", 1)]
    trace4 = [Acquire("t1", 1), Report("t1", 1), Write("t1", "x"), Release("t1", 1)]
    trace5 = [Report("t1", 1)]
    trace6 = []
    monitor = AllMonitors()
    for event in trace2:
        monitor.eval(event)
    monitor.end()
