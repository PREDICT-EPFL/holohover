"""
Worker processes for the outer updates (Section 8): the bounce sequences are solved in
parallel, one task per sequence (shot_selector.solve_sequence).

Processes, not threads: IPOPT 3.14 serialises every call into MUMPS with a process-wide
mutex, so threads queue on it (measured: 4 threads slower than 1), and one CasADi solver
called from several threads crashes. Each forked worker has its own copy of the built
solvers and of the MUMPS lock.

Create the pool before JAX starts its threads (physics_from_config): forking a
multithreaded process can deadlock the child. ProcessPoolExecutor forks all its workers at
the first submit, so the pool submits a no-op at creation. It never forks again: a dead
worker breaks the pool, and the solves then run serially in the parent.

The solves are deterministic, so the results are bit-identical to serial solves.
"""

import multiprocessing as mp
import os
import signal
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool

_NLP = None  # the ShotNLP of the pool being created, inherited by its forked workers


def _init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C is for the parent, which shuts the pool down


def _call(fn, args):
    return fn(_NLP, *args)


def _pid(nlp):
    return os.getpid()


class SolvePool:
    def __init__(self, nlp, n_workers):
        """n_workers forked copies of nlp, at most one per CPU this process may use."""
        global _NLP
        _NLP = nlp
        self.nlp = nlp
        self.n_workers = max(1, min(n_workers, len(os.sched_getaffinity(0))))
        self._ex = ProcessPoolExecutor(self.n_workers, mp_context=mp.get_context("fork"),
                                       initializer=_init_worker)
        self._ex.submit(_call, _pid, ()).result()  # fork every worker now

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
