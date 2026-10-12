#!/usr/bin/env python3
"""One way for a check to say what it found.

Five files carried their own `verdict()` and they had already drifted three
ways: sitl_check.py appended `(name, ok)` where the others appended a bare
bool, and two of the five had no `flush=True`. That last one is not cosmetic.
Every one of these runs under gz_run.sh with stdout redirected to a file, so
Python block-buffers it, and a run that is killed - or that wedges in the
simulator, which is how most of them end - loses whatever verdicts were still
in the buffer. The output then stops mid-check and reads as if the check hung
where it was actually several verdicts further on.

So: one implementation, flushed, and a summary that the caller returns.
"""

import os
import sys

_results = []
_broken = False

# Set to a column width for checks that print a table of short facts
# (cloudship_geom_check). Left at 0 the name and the detail are joined with a
# dash, which reads better for the one-sentence verdicts everything else has.
ALIGN = 0


def _write(line):
    """Print, and keep going when the reader has stopped reading.

    Someone pipes a check into `head` to see the first few verdicts. Catching
    the BrokenPipeError is only half of it: the interpreter flushes stdout
    again at exit and raises there too, outside any handler, so the run ends
    in a traceback about the pipe rather than with its output. Point stdout at
    the void once the pipe is gone and the final flush has somewhere to go.
    """
    global _broken

    if _broken:
        return

    try:
        print(line, flush=True)
    except BrokenPipeError:
        _broken = True
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())


def verdict(name, ok, detail):
    """Record and print one pass/fail. `ok` is coerced, so a None means fail."""
    ok = bool(ok)
    _results.append((name, ok))

    if ALIGN:
        _write('  %s  %-*s %s' % ('통과' if ok else '실패', ALIGN, name, detail))
    else:
        _write('  %s  %s — %s' % ('통과' if ok else '실패', name, detail))

    return ok


def note(text):
    """A line that is not a verdict: a measured baseline, a heading, a warning."""
    _write(text)


def results():
    """[(name, ok)] in the order they were recorded."""
    return list(_results)


def summary():
    """Print the tally and return the exit code the check should use."""
    good = sum(1 for _, ok in _results if ok)
    _write('통과 %d / 실패 %d' % (good, len(_results) - good))
    return 0 if good == len(_results) else 1
