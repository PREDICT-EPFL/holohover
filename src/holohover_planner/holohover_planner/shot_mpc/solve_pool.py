"""
Worker processes for the outer updates (Section 8): the bounce sequences are solved in
parallel, one task per sequence (shot_selector.solve_sequence).

Processes, not threads: IPOPT 3.14 serialises every call into MUMPS with a process-wide
mutex, so threads queue on it (measured: 4 threads slower than 1), and one CasADi solver
called from several threads crashes.

The workers are spawned, not forked: the node creates the pool after rclpy has started its
threads, and forking a multithreaded process can deadlock the child. Each worker builds its
own ShotNLP from the parameters at startup (~1 s). The pool starts every worker at creation
and never starts one again: a dead worker breaks the pool, and the solves then run serially
in the parent.

The solves are deterministic, so the results are bit-identical to serial solves.
"""

import multiprocessing as mp
import os
import signal
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool

_NLP = None  # the ShotNLP of this worker process


def _init_worker(P):
    global _NLP
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C is for the parent, which shuts the pool down
    from .shot_nlp import ShotNLP
    _NLP = ShotNLP(P, verbose=False)


def _call(fn, args):
    return fn(_NLP, *args)


def _pid(nlp):
    return os.getpid()


class SolvePool:
    def __init__(self, nlp, n_workers):
        """n_workers processes with their own copy of nlp, at most one per CPU this process may use."""
        self.nlp = nlp
        self.n_workers = max(1, min(n_workers, len(os.sched_getaffinity(0))))
        self._ex = ProcessPoolExecutor(self.n_workers, mp_context=mp.get_context("spawn"),
                                       initializer=_init_worker, initargs=(nlp.P,))
        # Start every worker now: one task each, all running at once
        list(self._ex.map(_call, [_pid] * (4 * self.n_workers), [()] * (4 * self.n_workers)))

    def map(self, fn, tasks):
        """[fn(nlp, *task) for task in tasks], computed in the workers."""
        if self._ex is not None:
            try:
                return list(self._ex.map(_call, [fn] * len(tasks), tasks))
            except BrokenProcessPool as err:
                print(f"[solve_pool] a worker died ({err}); solving serially from now on", flush=True)
                self._ex = None
        return [fn(self.nlp, *task) for task in tasks]

    def close(self):
        if self._ex is not None:
            self._ex.shutdown(cancel_futures=True)
            self._ex = None
