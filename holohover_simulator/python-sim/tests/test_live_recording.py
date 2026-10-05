"""run_live.py recording: an episode saved mid-recording is a finished, indexed MCAP; temporary files go."""

import json

import foxglove
from foxglove import messages as fm
from foxglove.channels import LogChannel
from mcap.reader import make_reader

from run_live import Recorder, write_indexed

N = 3000  # messages per channel: several chunks
STEP_NS = 10_000_000


def log_range(log, raw, ks):
    for k in ks:
        log.log(fm.Log(message=f"m{k}", name="test"), log_time=k * STEP_NS)
        raw.log({"k": k}, log_time=k * STEP_NS)


def test_save_while_recording(tmp_path):
    ctx = foxglove.Context()
    recorder = Recorder(ctx)
    log = LogChannel("/events", context=ctx)
    raw = foxglove.Channel("/raw", message_encoding="json", context=ctx)

    try:
        recorder.start("episode0")
        log_range(log, raw, range(N))
        saved = tmp_path / "saved.mcap"
        with open(saved, "wb") as f:
            n = write_indexed(recorder.snapshot(), f, {"start": "scenario test", "t_end": 1.5},
                              [("config.json", "application/json", b"{}", 0)])
        assert n == 2 * N

        with open(saved, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary.statistics.message_count == 2 * N
            assert len(summary.chunk_indexes) > 0
            assert {c.topic for c in summary.channels.values()} == {"/events", "/raw"}
            assert summary.statistics.message_end_time == (N - 1) * STEP_NS
            assert {m.name: m.metadata for m in reader.iter_metadata()} == {
                "holohover": {"start": "scenario test", "t_end": "1.5"}}
            assert [a.name for a in reader.iter_attachments()] == ["config.json"]
            last = list(reader.iter_messages(topics=["/raw"], start_time=(N - 1) * STEP_NS))
            assert [json.loads(m.data) for _, _, m in last] == [{"k": N - 1}]

        # The recording goes on after a save, and a new episode replaces its file
        log_range(log, raw, range(N, N + 10))
        first = recorder.path
        recorder.start("episode1")
        assert not first.exists() and recorder.path.exists()
    finally:
        recorder.close()
    assert not recorder.dir.exists()
