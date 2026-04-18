"""Lightweight phase-timing profiler for import / facade pipelines.

Usage
-----

```python
from flight_planner._profiling import phase, timed, start_recording, get_recorded

start_recording()                 # clear thread-local state for this request

with phase("parse_kmz"):
    parsed = parse_kmz(data)

@timed("load_pointcloud")         # decorator form
def load_pointcloud(...): ...

timings = get_recorded()          # [{"label": "...", "seconds": 0.12}, ...]
```

All state is stored in a :class:`threading.local` so concurrent requests
don't interleave. There is no lock, no IPC, no process-wide aggregation —
this is a dev tool, not an APM.
"""

from __future__ import annotations

import functools
import threading
import time
from contextlib import contextmanager
from typing import Callable, Iterator, TypeVar

_state = threading.local()
F = TypeVar("F", bound=Callable[..., object])


def _ensure() -> list[dict[str, object]]:
    if not hasattr(_state, "phases"):
        _state.phases = []
    return _state.phases


def start_recording() -> None:
    """Reset the thread-local phase list. Call at the start of a request."""
    _state.phases = []


def get_recorded() -> list[dict[str, object]]:
    """Snapshot the thread-local phase list as a plain JSON-serialisable list."""
    return list(_ensure())


@contextmanager
def phase(label: str) -> Iterator[None]:
    """Record the wall-clock duration of the enclosed block under ``label``."""
    t0 = time.perf_counter()
    try:
        yield
    finally:
        _ensure().append({
            "label": label,
            "seconds": round(time.perf_counter() - t0, 4),
        })


def timed(label: str | None = None) -> Callable[[F], F]:
    """Decorator form of :func:`phase`. Uses ``func.__name__`` if label omitted."""
    def wrap(fn: F) -> F:
        lbl = label or fn.__name__

        @functools.wraps(fn)
        def inner(*args: object, **kwargs: object) -> object:
            with phase(lbl):
                return fn(*args, **kwargs)

        return inner  # type: ignore[return-value]

    return wrap
