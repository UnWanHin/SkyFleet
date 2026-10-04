"""黄金摘要：仿真结果的**逐字节**回归守卫。

为什么值得用一条会"无故变红"的测试来卡住：
本项目的全部价值在于"同一份输入永远给出同一份输出"。一旦有人在重构中
无意改动了任何影响结果的细节——多消费了一次随机数、改了事件撰写顺序、
调整了某个阈值——两次运行仍然一致，所有单元测试也仍然全绿，
但**已经写进报告的数字全部作废**。普通的一致性测试抓不到这种情况，
只有把某个具体场景的结果摘要钉死才抓得到。

失败时怎么办：
1. 先确认这次改动**确实是有意**影响结果的（而不是顺手重构带出来的）。
2. 在 `docs/superpowers/specs/` 的设计修正记录里写明改了什么、为什么。
3. 再把下面的 `GOLDEN` 换成新值。

顺序不能颠倒：先更新常量再想理由，等于把这条测试删掉。
"""
import os
import tempfile
import unittest
from dataclasses import replace

from skyfleet.kernel.config import baseline_path, load_config
from skyfleet.observe.log import normalised_digest
from skyfleet.sim.engine import Engine

#: 基线场景（scenarios/baseline_small.json，种子 1，600 s）的规范化日志摘要。
#:
#: 变更记录（每次更新都必须在此留下原因）：
#: - 2026-10-04 时延六段分解修正：未在本机入队的消息不再记 0.0 减生成时刻。
#: - 2026-10-04 P1 注入接口：SAFETY_ACTION 事件增加 geofence 与 position_age
#:   两个**观测字段**。仿真行为未变（事件总数、完成率、丢包率、时延分位数
#:   全部逐项一致），仅日志内容增加。
#: - 2026-10-04 P1 安全动作可观测性：SAFETY_ACTION 增加 cause 字段，记录
#:   触发原因而不只是动作；原因参与变化检测，使"已在悬停、随后 GNSS 才
#:   失锁"这类转换不再被漏记。行为仍未变：事件总数、完成率、丢包率、
#:   送达率与丢弃原因分布逐项一致。
#: - 2026-10-04 P1 接收端有效期校验：TASK_ASSIGN / TASK_REASSIGN 在投递时
#:   若已过 deadline 判 DEADLINE_MISSED。基线场景该分支命中 0 次，
#:   各项指标与丢弃原因分布完全未变；它是为对抗注入准备的防线。
GOLDEN = "aff4a602142b20c55fec107a7ebb1e7b8f115e4505490576c70254f4b259a854"


class TestGoldenDigest(unittest.TestCase):
    def test_baseline_matches_the_recorded_digest(self):
        config = load_config(baseline_path())
        with tempfile.TemporaryDirectory() as tmp:
            config = replace(config, log_path=os.path.join(tmp, "b.jsonl"))
            Engine(config).run()
            digest = normalised_digest(config.log_path)

        self.assertEqual(
            digest, GOLDEN,
            "\n基线场景的结果摘要变了。\n"
            "若这是有意为之：在设计修正记录中写明原因，再更新 GOLDEN。\n"
            "若并非有意：说明有改动影响了仿真结果，报告中已有数字需要复核。\n"
            f"  实际：{digest}\n  期望：{GOLDEN}")

    def test_environment_seed_does_not_perturb_the_result(self):
        """显式固定 env_seed 与让它退回 seed 必须完全等价。

        这条守的是"把环境随机源拆出来"这个改动本身：如果拆分时不小心
        多消费或少消费了随机数，结果就会漂，而漂移方向未必看得出来。
        """
        with tempfile.TemporaryDirectory() as tmp:
            digests = []
            for index, env_seed in enumerate((None, 1)):
                config = load_config(baseline_path())
                config = replace(config, env_seed=env_seed,
                                 log_path=os.path.join(tmp, f"{index}.jsonl"))
                Engine(config).run()
                digests.append(normalised_digest(config.log_path))
        self.assertEqual(digests[0], digests[1])


if __name__ == "__main__":
    unittest.main()
