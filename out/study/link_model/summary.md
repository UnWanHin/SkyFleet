# 扫参结果：link_model

链路模型对比：P0 几何遮挡（二值）vs P1 绕射+快衰落（连续）。两者在同一城市、同一协议下对比，用于回答「二值遮挡模型会把什么结论弄错」。

- 基准场景：`skyfleet/scenarios/baseline_small.json`
- 主种子：1，每个组合重复 3 次，共 6 次运行
- 参数网格：`{"link_model": ["geometric", "nlos"]}`

| 组合 | link_model | 重复 | tasks.completion_rate（均值±95%CI） | delivery_confirm.success_rate（均值±95%CI） | latency_ms.Q1.mean（均值±95%CI） | latency_ms.Q1.p95（均值±95%CI） | latency_ms.Q2.mean（均值±95%CI） | latency_ms.Q2.p95（均值±95%CI） | latency_ms.Q3.mean（均值±95%CI） | latency_ms.Q3.p95（均值±95%CI） | loss.control_rate（均值±95%CI） | loss.data_rate（均值±95%CI） | drop_reasons.COLLISION（均值±95%CI） | drop_reasons.OCCLUDED（均值±95%CI） | heartbeat.delivery_rate（均值±95%CI） | retransmission.total（均值±95%CI） | retransmission.expired（均值±95%CI） | route_switches（均值±95%CI） | forwarded（均值±95%CI） | recovery.mean_s（均值±95%CI） | recovery.recovered（均值±95%CI） | overhead.total_bytes（均值±95%CI） | overhead.bytes_per_delivered_task（均值±95%CI） | energy.min_final（均值±95%CI） |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | geometric | 3 | 1 ± 0 | 0.9841 ± 0.068 | 24.73 ± 71 | 1.968 ± 0.00091 | 1.313 ± 0.0046 | 2.21 ± 0.013 | 1.354 ± 0.021 | 2.941 ± 0.054 | 0.05883 ± 0.025 | 0.05432 ± 0.011 | 1083 ± 2.5e+02 | 134.7 ± 18 | 0.9943 ± 0.00015 | 0 ± 0 | 57.33 ± 1.4 | 5 ± 0 | 1560 ± 24 | — | 0 ± 0 | 1.027e+06 ± 1.4e+04 | 5.133e+04 ± 6.8e+02 | 69.32 ± 0.0038 |
| 1 | nlos | 3 | 1 ± 0 | 0.9697 ± 0.13 | 30.56 ± 69 | 1.968 ± 0 | 1.226 ± 0.051 | 2.125 ± 0.036 | 1.2 ± 0.037 | 2.1 ± 0.032 | 0.04735 ± 0.0055 | 0.05796 ± 0.01 | 901.7 ± 1e+02 | 302 ± 60 | 0.9935 ± 0.0012 | 0 ± 0 | 57.67 ± 2.9 | 2.333 ± 2.9 | 64 ± 2.6e+02 | — | 0 ± 0 | 9.536e+05 ± 6.1e+03 | 4.768e+04 ± 3.1e+02 | 69.62 ± 0.039 |

> 区间为双尾 95% t 置信区间。`—` 表示该组合下没有该指标的样本（例如没有节点恢复时的平均恢复时间），不是 0。
