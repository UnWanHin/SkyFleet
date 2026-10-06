# 扫参结果：energy_model

能量模型对比：linear（与飞行状态无关的恒定扣费，P0/P1 行为）vs physical（四旋翼动量理论，按悬停/巡航/爬升/下降计费）。用于回答「恒定时钟把什么结论弄错」，并量化被迫悬停的代价。协议层零改动——换模型不影响交付率与丢失率。

- 基准场景：`skyfleet/scenarios/baseline_small.json`
- 主种子：1，每个组合重复 3 次，共 6 次运行
- 参数网格：`{"energy_model": ["linear", "physical"]}`

| 组合 | energy_model | 重复 | tasks.completion_rate（均值±95%CI） | delivery_confirm.success_rate（均值±95%CI） | latency_ms.Q1.mean（均值±95%CI） | latency_ms.Q1.p95（均值±95%CI） | latency_ms.Q2.mean（均值±95%CI） | latency_ms.Q2.p95（均值±95%CI） | latency_ms.Q3.mean（均值±95%CI） | latency_ms.Q3.p95（均值±95%CI） | loss.control_rate（均值±95%CI） | loss.data_rate（均值±95%CI） | drop_reasons.COLLISION（均值±95%CI） | drop_reasons.OCCLUDED（均值±95%CI） | heartbeat.delivery_rate（均值±95%CI） | retransmission.total（均值±95%CI） | retransmission.expired（均值±95%CI） | route_switches（均值±95%CI） | forwarded（均值±95%CI） | recovery.mean_s（均值±95%CI） | recovery.recovered（均值±95%CI） | overhead.total_bytes（均值±95%CI） | overhead.bytes_per_delivered_task（均值±95%CI） | energy.min_final（均值±95%CI） |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | linear | 3 | 1 ± 0 | 0.9841 ± 0.068 | 24.73 ± 71 | 1.968 ± 0.00091 | 1.313 ± 0.0046 | 2.21 ± 0.013 | 1.354 ± 0.021 | 2.941 ± 0.054 | 0.05883 ± 0.025 | 0.05432 ± 0.011 | 1083 ± 2.5e+02 | 134.7 ± 18 | 0.9943 ± 0.00015 | 0 ± 0 | 57.33 ± 1.4 | 5 ± 0 | 1560 ± 24 | — | 0 ± 0 | 1.027e+06 ± 1.4e+04 | 5.133e+04 ± 6.8e+02 | 69.32 ± 0.0038 |
| 1 | physical | 3 | 1 ± 0 | 0.9697 ± 0.13 | 14.68 ± 31 | 1.968 ± 0 | 1.309 ± 0.036 | 2.212 ± 0.011 | 1.351 ± 0.019 | 2.959 ± 0.12 | 0.04823 ± 0.0073 | 0.05422 ± 0.0068 | 1046 ± 1e+02 | 135 ± 5 | 0.9944 ± 0.0017 | 0 ± 0 | 57.67 ± 2.9 | 5 ± 0 | 1553 ± 41 | — | 0 ± 0 | 1.028e+06 ± 7.1e+03 | 5.142e+04 ± 3.6e+02 | 69.03 ± 0.0073 |

> 区间为双尾 95% t 置信区间。`—` 表示该组合下没有该指标的样本（例如没有节点恢复时的平均恢复时间），不是 0。
