"""事件日志测试。

日志是唯一事实来源：指标与回放都从它重算。因此
"同一内容摘要稳定、内容变化摘要改变"必须被钉死。
"""
import tempfile
import unittest
from pathlib import Path

from skyfleet.observe.events import DropReason, EventKind
from skyfleet.observe.log import EventLog, normalised_digest, read_log


class TestWriteRead(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "log.jsonl"

    def tearDown(self):
        self._tmp.cleanup()

    def _write_sample(self):
        with EventLog(self.path) as log:
            log.write(EventKind.NODE_JOIN, 0.0, node_id="S1", role="SLAVE")
            log.write(EventKind.MSG_DROP, 1.5, node_id="S1", msg_id="m1",
                      reason=DropReason.OCCLUDED)

    def test_records_are_readable(self):
        self._write_sample()
        records = list(read_log(self.path))
        self.assertEqual(len(records), 2)

    def test_kind_is_serialised_as_string(self):
        self._write_sample()
        self.assertEqual(list(read_log(self.path))[0]["kind"], "NODE_JOIN")

    def test_extra_fields_are_preserved(self):
        self._write_sample()
        first = list(read_log(self.path))[0]
        self.assertEqual(first["node_id"], "S1")
        self.assertEqual(first["role"], "SLAVE")

    def test_enum_reason_serialised_as_string(self):
        self._write_sample()
        second = list(read_log(self.path))[1]
        self.assertEqual(second["reason"], "OCCLUDED")

    def test_timestamp_is_preserved(self):
        self._write_sample()
        self.assertAlmostEqual(list(read_log(self.path))[1]["t"], 1.5)

    def test_empty_log_reads_as_nothing(self):
        self.path.write_text("", encoding="utf-8")
        self.assertEqual(list(read_log(self.path)), [])

    def test_missing_log_raises(self):
        with self.assertRaises(FileNotFoundError):
            list(read_log(self.path))

    def test_appends_across_sessions(self):
        self._write_sample()
        with EventLog(self.path) as log:
            log.write(EventKind.NODE_LEAVE, 9.0, node_id="S1")
        self.assertEqual(len(list(read_log(self.path))), 3)


class TestDigest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "log.jsonl"

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, drop_reason=DropReason.OCCLUDED):
        with EventLog(self.path) as log:
            log.write(EventKind.NODE_JOIN, 0.0, node_id="S1")
            log.write(EventKind.MSG_DROP, 1.5, node_id="S1",
                      reason=drop_reason)

    def test_same_content_gives_same_digest(self):
        self._write()
        first = normalised_digest(self.path)
        second = normalised_digest(self.path)
        self.assertEqual(first, second)

    def test_digest_is_sha256_hex(self):
        self._write()
        digest = normalised_digest(self.path)
        self.assertEqual(len(digest), 64)
        int(digest, 16)

    def test_content_change_changes_digest(self):
        self._write(DropReason.OCCLUDED)
        a = normalised_digest(self.path)
        self._write(DropReason.COLLISION)
        b = normalised_digest(self.path)
        self.assertNotEqual(a, b)

    def test_field_order_does_not_matter(self):
        with EventLog(self.path) as log:
            log.write(EventKind.MSG_TX, 1.0, a=1, b=2)
        first = normalised_digest(self.path)
        self.path.unlink()
        with EventLog(self.path) as log:
            log.write(EventKind.MSG_TX, 1.0, b=2, a=1)
        self.assertEqual(first, normalised_digest(self.path))


if __name__ == "__main__":
    unittest.main()
