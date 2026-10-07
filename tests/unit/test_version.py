"""版本、任期与去重接受规则测试（纯函数）。"""
import unittest

from skyfleet.protocol.message import Msg
from skyfleet.protocol.version import (AcceptDecision, DedupTable, LocalView,
                                       accept)


def make(**kw):
    base = dict(msg_type="TASK_ASSIGN", src="L", dst="S1", t=0.0)
    base.update(kw)
    return Msg.new(**base)


class TestAcceptRules(unittest.TestCase):
    def setUp(self):
        self.view = LocalView(leader_epoch=2, mission_version=5,
                              route_version=1, sessions={"L#1"})
        self.dedup = DedupTable(ttl=10.0, capacity=100)

    def test_accepts_fresh_message(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_old_epoch(self):
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_accepts_newer_epoch(self):
        m = make(leader_epoch=3, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_rejects_unknown_session(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SESSION)

    def test_rejects_old_mission_version(self):
        m = make(leader_epoch=2, mission_version=4, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_VERSION)

    def test_epoch_beats_session_check(self):
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#0")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_EPOCH)

    def test_rejects_duplicate_operation(self):
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#1",
                 operation_id="op-7")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)
        self.assertIs(accept(m, self.view, self.dedup, 0.1), AcceptDecision.DEDUP)

    def test_rejects_stale_seq(self):
        self.view.last_seq[("L", "TASK_ASSIGN", 2, 5)] = 9
        m = make(leader_epoch=2, mission_version=5, seq=9, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.STALE_SEQ)

    def test_accepts_higher_seq(self):
        self.view.last_seq[("L", "TASK_ASSIGN", 2, 5)] = 9
        m = make(leader_epoch=2, mission_version=5, seq=10, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_seq_is_scoped_per_message_stream(self):
        """序列号只在同一源的同一条消息流内可比。

        心跳与遥测共用一个发送计数器，但互不排序。若把 msg_type 排除在
        序列键外，一条乱序到达的遥测就会让紧随其后的心跳被判为过期，
        凭空制造一次"心跳丢失"，进而把健康节点误判为故障。
        """
        self.view.last_seq[("L", "TELEMETRY", 2, 5)] = 99
        m = make(leader_epoch=2, mission_version=5, seq=1, session_id="L#1")
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_status_messages_ignore_mission_version(self):
        """心跳/遥测不属于任何任务计划，不得受 mission_version 约束。

        否则一次任务重派就会让尚未收到新版本的节点心跳全部失效。
        """
        self.view.mission_version = 9
        m = Msg.new("HEARTBEAT", src="L", dst="S1", t=0.0, seq=1,
                    session_id="L#1", leader_epoch=2, mission_version=0)
        self.assertIs(accept(m, self.view, self.dedup, 0.0), AcceptDecision.ACCEPT)

    def test_accept_updates_view(self):
        m = make(leader_epoch=4, mission_version=7, seq=3, session_id="L#1")
        accept(m, self.view, self.dedup, 0.0)
        self.assertEqual(self.view.leader_epoch, 4)
        self.assertEqual(self.view.mission_version, 7)
        self.assertEqual(self.view.last_seq[("L", "TASK_ASSIGN", 4, 7)], 3)

    def test_reject_does_not_mutate_view(self):
        before = (self.view.leader_epoch, self.view.mission_version)
        m = make(leader_epoch=1, mission_version=5, seq=1, session_id="L#1")
        accept(m, self.view, self.dedup, 0.0)
        self.assertEqual((self.view.leader_epoch, self.view.mission_version), before)


class TestDedupTable(unittest.TestCase):
    def test_first_sight_is_not_duplicate(self):
        d = DedupTable(ttl=5.0, capacity=10)
        self.assertFalse(d.seen("op", 0.0))

    def test_repeat_within_ttl_is_duplicate(self):
        d = DedupTable(ttl=5.0, capacity=10)
        d.seen("op", 0.0)
        self.assertTrue(d.seen("op", 4.9))

    def test_expires_after_ttl(self):
        d = DedupTable(ttl=5.0, capacity=10)
        d.seen("op", 0.0)
        self.assertFalse(d.seen("op", 5.1))

    def test_capacity_is_bounded(self):
        d = DedupTable(ttl=1000.0, capacity=3)
        for i in range(50):
            d.seen(f"op{i}", float(i))
        self.assertLessEqual(len(d), 3)

    def test_prune_drops_expired(self):
        d = DedupTable(ttl=1.0, capacity=100)
        for i in range(10):
            d.seen(f"op{i}", 0.0)
        d.prune(5.0)
        self.assertEqual(len(d), 0)


class TestSeqSlidingWindow(unittest.TestCase):
    """序号必须是**滑动窗口**，而不是无上界的单调比较。

    原实现只有 `view.last_seq[key] = msg.seq`，没有任何范围检查。冒充者
    发送**一个** seq = 10^7 的包，就能把接收端的 last_seq 推到 10^7，
    此后所有真实节点的心跳（seq 只有几十）都满足 seq <= previous，
    全部被判 STALE_SEQ 丢弃——**单封包拒绝服务**。

    结果章节已把该缺口如实写入并声明「修复方向是滑动窗口」，本类钉死修复。
    """

    #: 规格 §1.3 定的默认窗口宽度。
    W = 64

    def _msg(self, seq: int, msg_type: str = "HEARTBEAT") -> Msg:
        # 用 Msg.new 构造：它按消息类型填好 qos/rel/deadline。直接调
        # Msg(...) 会漏掉这些字段，也早就不是本仓库的用法。
        return Msg.new(msg_type=msg_type, src="S1", dst="L", t=0.0, seq=seq)

    def _view(self) -> LocalView:
        return LocalView()

    def test_a_single_forged_sequence_cannot_blind_the_receiver(self):
        """★ 核心回归：这是原缺口的直接复现。

        伪造一个极大的序号之后，真实心跳**必须仍然被接受**。
        """
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        # 现实情境：被冒充的节点一直在发心跳，接收端**已有基准**。
        # 冒充者不可能"先于"它冒充的节点出现——它冒充的正是那条已存在的流。
        accept(self._msg(1), view, dedup, 0.0)
        forged = self._msg(10_000_000)
        decision = accept(forged, view, dedup, 0.1)
        self.assertIs(decision, AcceptDecision.SEQ_OUT_OF_WINDOW,
                      "极大的伪造序号必须被单独判为越窗，而不是被接受")
        # 关键断言：真实心跳不受影响。修复前这里会全部变成 STALE_SEQ。
        for seq in (2, 3, 4, 5, 6):
            with self.subTest(seq=seq):
                self.assertIs(accept(self._msg(seq), view, dedup, float(seq)),
                              AcceptDecision.ACCEPT,
                              "伪造序号把接收端打瞎了——真实心跳被误判为过期")

    def test_a_first_arriving_spoofer_is_a_known_residual_gap(self):
        """★ 已知残差：**先到的**冒充者防不住，滑动窗口不是认证。

        若接收端从未见过这条流，就没有基准可以判断 10^7 是否离谱，
        它会被当作锚点。此后真实心跳确实会被误判为过期。

        这不是本切片的实现缺陷，而是「无认证的序号窗口」的本质边界：
        要挡住先到的冒充者，只能靠**认证**（MAC）。

        **P4 已提供认证**（`docs/superpowers/specs/2026-10-09-p4-message-
        authentication-spec.md`）：`auth_enabled=True` 时冒充者在认证处即被
        拒，残差关闭；本测试验证的是**默认（无认证）**下残差仍存在——这
        是刻意的默认值（新机制默认不改变既有行为）。两侧的完整对照见
        `tests/unit/test_auth.py` 的 `TestFirstArrivingSpoofer`。

        本测试把残差**固定下来**，避免有人误以为滑动窗口本身已经安全。
        """
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        # 冒充者先到（接收端无基准）
        self.assertIs(accept(self._msg(10_000_000), view, dedup, 0.0),
                      AcceptDecision.ACCEPT,
                      "无基准时无法拒绝——这是默认（未启用认证）下的既定"
                      "行为。若这里变成越窗，说明默认值被改了，golden "
                      "digest 也会随之变化")
        # 真实心跳因此被误判
        self.assertIs(accept(self._msg(1), view, dedup, 1.0),
                      AcceptDecision.STALE_SEQ,
                      "先到的冒充者仍然能打瞎接收端——这是已知残差")

    def test_an_out_of_window_packet_does_not_move_the_window(self):
        """★ 若越窗包推进了窗口，冒充者只要连发 last_seq+W 就能走到任意远处。"""
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        accept(self._msg(1), view, dedup, 0.0)          # 正常起点 seq=1
        self.assertIs(accept(self._msg(10_000_000), view, dedup, 1.0),
                      AcceptDecision.SEQ_OUT_OF_WINDOW)
        # 窗口必须仍锚在 1。W+2 = 66 越窗；若窗口被推到 10^7，
        # 下面这包会被接受（因为 66 > 10^7 不成立）——用这一点抓回归。
        self.assertIs(accept(self._msg(self.W + 2), view, dedup, 2.0),
                      AcceptDecision.SEQ_OUT_OF_WINDOW,
                      "越窗包推进了窗口，冒充者可逐步走到任意远处")

    def test_the_window_admits_exactly_W_of_forward_jump(self):
        """窗口宽度必须恰好是 W：W 内的跃变接受，W+1 拒绝。

        实测边界防止有人把 `>` 写成 `>=`（那会让窗口少一格），
        或把 W 写成无穷（那就回到了无上界）。
        """
        key_start = 10
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        accept(self._msg(key_start), view, dedup, 0.0)
        self.assertIs(accept(self._msg(key_start + self.W), view, dedup, 1.0),
                      AcceptDecision.ACCEPT, f"跃变恰好 {self.W} 应当接受")
        # 上一包把锚点推到 key_start+W，故再 +W+1 越窗
        self.assertIs(
            accept(self._msg(key_start + self.W + self.W + 1), view, dedup, 2.0),
            AcceptDecision.SEQ_OUT_OF_WINDOW, f"跃变 {self.W + 1} 应当拒绝")

    def test_in_window_out_of_order_is_still_accepted(self):
        """窗口内的乱序（前向）必须被接受——否则修 DoS 会换来丢包率上升。"""
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        accept(self._msg(100), view, dedup, 0.0)
        for seq in range(101, 101 + self.W):
            with self.subTest(seq=seq):
                self.assertIs(accept(self._msg(seq), view, dedup, float(seq)),
                              AcceptDecision.ACCEPT,
                              "窗口内的前向乱序被拒绝了")

    def test_a_backward_sequence_is_still_stale(self):
        """回退仍走 STALE_SEQ（原行为不变），且不移动窗口。"""
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        accept(self._msg(100), view, dedup, 0.0)
        self.assertIs(accept(self._msg(99), view, dedup, 1.0),
                      AcceptDecision.STALE_SEQ)
        self.assertIs(accept(self._msg(100), view, dedup, 2.0),
                      AcceptDecision.STALE_SEQ)
        self.assertIs(accept(self._msg(101), view, dedup, 3.0),
                      AcceptDecision.ACCEPT, "回退不应把窗口往回拖")

    def test_the_window_is_scoped_per_stream(self):
        """窗口按流分开，一条流的越窗不影响另一条流。"""
        view, dedup = self._view(), DedupTable(ttl=10.0, capacity=100)
        accept(self._msg(1), view, dedup, 0.0)
        accept(self._msg(1, msg_type="TELEMETRY"), view, dedup, 0.0)
        self.assertIs(accept(self._msg(10_000_000), view, dedup, 1.0),
                      AcceptDecision.SEQ_OUT_OF_WINDOW)
        self.assertIs(accept(self._msg(2, msg_type="TELEMETRY"), view, dedup, 2.0),
                      AcceptDecision.ACCEPT,
                      "另一条消息流被连坐")

    def test_the_reason_code_is_in_the_closed_enum(self):
        """越窗必须是**独立**原因码：与 STALE_SEQ 混在一起就无法观测攻击。

        本项目已经犯过一次同类错误——`STALE_SEQ` 会被写进日志却不在
        `DropReason` 枚举内，「封闭枚举」在代码层面是假的。
        """
        from skyfleet.observe.events import DropReason
        self.assertIn(AcceptDecision.SEQ_OUT_OF_WINDOW.value,
                      {r.value for r in DropReason},
                      "越窗原因码不在封闭枚举 DropReason 内")
        self.assertNotEqual(AcceptDecision.SEQ_OUT_OF_WINDOW,
                            AcceptDecision.STALE_SEQ,
                            "越窗不能与过期共用一个原因码，否则攻击不可观测")


if __name__ == "__main__":
    unittest.main()
