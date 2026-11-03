"""Outcome of one navigation attempt, for callers that must decide what to do next."""
from typing import NamedTuple


class NavResult(NamedTuple):
    """outcome is SUCCEEDED, FAILED, CANCELED or NOT_READY (never started)."""

    outcome: str
    text: str
    seconds: float = 0.0

    @property
    def ok(self):
        return self.outcome == 'SUCCEEDED'
