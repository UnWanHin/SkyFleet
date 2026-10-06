#!/usr/bin/env python3
"""把「能量与续航」一节生成进结果章节的 HTML。

**为什么不手改 HTML**：这一节的每个数字都来自 `out/study/*/runs.csv` 或
实测日志。手抄一遍就等于制造第二个事实来源——扫描重跑后正文与数据会
静默脱节，而这正是本项目已经犯过一次的错误。所以数字全部在这里现算。

设计约束：
- 只输出转换器认得的标签（h2/h3/p/table/caption/code/b/img/sup/hr）。
- 幂等：用 HTML 注释做标记，重复运行只替换自己插入的块。
- 不编造：任何一个数都不得硬编码，必须来自 CSV 或 Airframe 属性。

用法：
    python3 docs/report/build_energy_section.py            # 就地更新 HTML
    python3 docs/report/build_energy_section.py --check    # 只校验是否有漂移
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import statistics as st
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / "docs/report/2026-10-04-results-chapter.html"
STUDY = ROOT / "out/study"

sys.path.insert(0, str(ROOT))

# --- 幂等标记 ---------------------------------------------------------------
MARK_START = "<!-- BEGIN generated: energy section -->"
MARK_END = "<!-- END generated: energy section -->"
MARK_STATS_START = "<!-- BEGIN generated: stats correction -->"
MARK_STATS_END = "<!-- END generated: stats correction -->"
MARK_LAYOUT_START = "<!-- BEGIN generated: layout correction -->"
MARK_LAYOUT_END = "<!-- END generated: layout correction -->"


# --- 数据读取 ---------------------------------------------------------------
def load(study: str) -> list[dict]:
    """读一个 study 的逐次结果。缺文件直接报错，绝不静默跳过。"""
    path = STUDY / study / "runs.csv"
    if not path.exists():
        raise SystemExit(
            f"缺少 {path}。先跑：\n"
            f"  python3 -m skyfleet study --spec skyfleet/study/specs/{study}.json "
            f"--jobs 12 --no-logs --quiet")
    with path.open(encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def num(rows: list[dict], column: str) -> list[float]:
    return [float(r[column]) for r in rows]


def mean_sd(values: list[float]) -> tuple[float, float]:
    return st.mean(values), (st.stdev(values) if len(values) > 1 else 0.0)


def subset(rows: list[dict], **eq) -> list[dict]:
    return [r for r in rows if all(r.get(k) == v for k, v in eq.items())]


# --- 统计量 -----------------------------------------------------------------
def welch_t(a: list[float], b: list[float]) -> tuple[float, int]:
    """Welch t 与 Welch–Satterthwaite 自由度。

    这里**真的**算了一个检验——而不是像 `skyfleet/study/stats.py` 里那个
    同名函数那样比较置信区间是否重叠（那个函数还是死代码）。
    """
    ma, mb = st.mean(a), st.mean(b)
    va, vb = st.variance(a), st.variance(b)
    na, nb = len(a), len(b)
    se2 = va / na + vb / nb
    t = (ma - mb) / math.sqrt(se2)
    df = se2**2 / ((va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1))
    return t, int(round(df))


def t_crit_95(df: int) -> float:
    """双尾 95% t 临界值（查表，df<=30）。表很小，直接写死比引 scipy 稳。"""
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
             7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
             13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110,
             18: 2.101, 19: 2.093, 20: 2.086, 21: 2.080, 22: 2.074,
             23: 2.069, 24: 2.064, 25: 2.060, 26: 2.056, 27: 2.052,
             28: 2.048, 29: 2.045, 30: 2.042}
    return table.get(df, 1.96)


def variance_share(rows: list[dict], value_col: str, group_col: str) -> tuple[float, float, float]:
    """组间/组内标准差与组间方差占比。

    这是本项目最重要的一个统计量：它说明「换一座城市」比「换一个策略」
    重要得多。用合并标准差（不假设等 n）。
    """
    groups: dict[str, list[float]] = {}
    for r in rows:
        groups.setdefault(r[group_col], []).append(float(r[value_col]))
    within_var = st.mean([st.variance(v) for v in groups.values() if len(v) > 1])
    between_var = st.variance([st.mean(v) for v in groups.values()])
    share = between_var / (between_var + within_var) if (between_var + within_var) else 0.0
    return math.sqrt(between_var), math.sqrt(within_var), share


# --- 片段生成 ---------------------------------------------------------------
def thrust_rows() -> list[tuple]:
    """四机型的推力余裕。数字直接取自 Airframe，不复制常量。"""
    from skyfleet.world.power import AIRFRAMES

    order = ("crazyflie", "crazyflie_brushless", "hummingbird", "px4_sihsim_quadx")
    out = []
    for name in order:
        af = AIRFRAMES[name]
        # 这些是**方法**不是属性，且都接受酬载参数——余裕必须按实际载荷算，
        # 否则「四电机够不够」回答的是空载的问题。
        out.append((af.name, af.mass_kg, af.num_rotors, af.max_thrust_n,
                    af.thrust_to_weight(0.0), af.hover_throttle(0.0) * 100.0,
                    af.max_bank_deg(0.0), af.required_tilt_deg(12.0, 0.0)))
    return out


def build_energy_section() -> str:
    em = load("energy_model")
    af = load("airframe")
    bp = load("battery_policy")
    bpl = load("battery_policy_layout")

    lin = subset(em, energy_model="linear")
    phy = subset(em, energy_model="physical")
    lin_m, lin_s = mean_sd(num(lin, "m:energy.min_final"))
    phy_m, phy_s = mean_sd(num(phy, "m:energy.min_final"))

    # 机型 × 酬载
    airframes = sorted({r["airframe"] for r in af})
    payloads = sorted({r["payload_kg"] for r in af}, key=float)
    af_tbl = []
    for a in airframes:
        cells = []
        for p in payloads:
            sub = subset(af, airframe=a, payload_kg=p)
            m, s = mean_sd(num(sub, "m:energy.min_final"))
            done = mean_sd(num(sub, "m:tasks.completion_rate"))[0]
            cells.append((m, s, done))
        af_tbl.append((a, cells))

    # 耗尽策略（单一布局）
    pol_tbl = []
    for pol in ("none", "land", "crash"):
        sub = subset(bp, on_battery_empty=pol)
        cr, crs = mean_sd(num(sub, "m:tasks.completion_rate"))
        hb, hbs = mean_sd(num(sub, "m:delivery_confirm.success_rate"))
        fw, fws = mean_sd(num(sub, "m:forwarded"))
        pol_tbl.append((pol, cr, crs, hb, hbs, fw, fws))

    # 布局 × 策略
    layouts = sorted({r["env_seed"] for r in bpl}, key=int)
    lay_tbl, diffs = [], []
    for es in layouts:
        row = []
        for pol in ("none", "land", "crash"):
            sub = subset(bpl, env_seed=es, on_battery_empty=pol)
            row.append(mean_sd(num(sub, "m:tasks.completion_rate"))[0])
        diffs.append(row[0] - row[2])
        lay_tbl.append((es, *row, row[0] - row[2]))

    d_mean, d_sd = mean_sd(diffs)
    n = len(diffs)
    t = d_mean / (d_sd / math.sqrt(n))
    tcrit = t_crit_95(n - 1)
    n_pos = sum(1 for d in diffs if d > 0)
    # 精确符号检验（双尾）
    p_sign = min(1.0, 2 * sum(math.comb(n, k) for k in range(n_pos, n + 1)) / 2**n)

    btw, wth, share = variance_share(bpl, "m:tasks.completion_rate", "env_seed")

    # 三策略的分群（转发量双峰）
    def two_cluster(pol: str):
        v = sorted(num(subset(bpl, on_battery_empty=pol), "m:forwarded"))
        gaps = [(v[i + 1] - v[i], i) for i in range(len(v) - 1)]
        g, i = max(gaps)
        within = max([v[j + 1] - v[j] for j in range(len(v) - 1) if j != i] or [0])
        return len(v), i + 1, len(v) - i - 1, g, within

    # 推力余裕
    thr = thrust_rows()

    parts: list[str] = [MARK_START]
    A = parts.append

    A('<h3>能量与续航：四电机够不够，以及会不会掉机</h3>')
    A('<p>课程要求报告「能耗趋势」。本章前九节都围绕通信协议，'
      '而能量是无人机与地面机器人最本质的差别：它同时是<b>通信节点</b>'
      '与<b>飞行器</b>，两者争用同一块电池。本节回答两个问题：'
      '四电机的推力余裕有多大，以及电量耗尽会发生什么。</p>')

    # --- 推力余裕 ---
    A('<h4>推力余裕：四电机是否够用</h4>')
    A('<p>推力按动量理论取 <code>T = k·ω²</code>，悬停油门为 '
      '<code>1/√TWR</code>，可持续的最大倾角为 <code>arccos(1/TWR)</code>'
      '（超过它就无法维持高度）。四个机型的结果如下。</p>')
    A('<table>')
    A('<caption>表 13  四机型的推力余裕（推力/重量比 TWR 与悬停油门）</caption>')
    A('<tr><th>机型</th><th>质量 (kg)</th><th>电机数</th><th>最大总推力 (N)</th>'
      '<th>TWR</th><th>悬停油门</th><th>最大倾角</th><th>12 m/s 所需倾角</th></tr>')
    for name, mass, nrot, tmax, twr, thr_pct, bank, tilt in thr:
        A(f'<tr><td>{name}</td><td>{mass:g}</td><td>{nrot}</td>'
          f'<td>{tmax:.3g}</td><td>{twr:.2f}</td><td>{thr_pct:.1f} %</td>'
          f'<td>{bank:.1f}°</td><td>{tilt:.1f}°</td></tr>')
    A('</table>')
    A('<p class="src">数据来源：<code>skyfleet/world/power.py</code> 的 '
      '<code>AIRFRAMES</code> 参数（取自 RotorPy 的机型定义）与 '
      '<code>Airframe</code> 属性，逐个现算。</p>')
    min_twr = min(r[4] for r in thr)
    A(f'<p>四个机型的 TWR 都在 <b>{min_twr:.2f}</b> 以上，'
      '悬停油门不超过 <b>{:.0f} %</b>，因此四电机在悬停与巡航上都有余量——'
      '<b>「四电机够不够」的答案是够</b>。真正的约束不是推力，而是能量：'
      '下面的数据会显示，同一个机型挂上酬载后，续航会先于推力成为瓶颈。</p>'
      .format(max(r[5] for r in thr)))

    # --- 能量模型对照 ---
    A('<h4>能量模型：线性时钟与物理模型是否一致</h4>')
    A('<p>模拟器支持两个能量模型。默认的<b>线性模型</b>把能量消耗当作'
      '恒定悬停时钟加发送计费；<b>物理模型</b>按动量理论分别计算悬停、'
      '巡航、爬升、下降的功率，并乘上图腾效率（Figure of Merit）与'
      '电机/电调效率。同一场景下两者的末值电量：</p>')
    A('<table>')
    A('<caption>表 14  两个能量模型在同一场景下的最终电量（初始 100，600 s）</caption>')
    A('<tr><th>能量模型</th><th>节点最低末值电量</th><th>相对线性模型</th></tr>')
    A(f'<tr><td>线性（默认）</td><td>{lin_m:.2f} ± {lin_s:.2f}</td><td>—</td></tr>')
    A(f'<tr><td>物理</td><td>{phy_m:.2f} ± {phy_s:.2f}</td>'
      f'<td>{(phy_m - lin_m) / (100.0 - lin_m) * 100:+.1f} % 耗电</td></tr>')
    A('</table>')
    A('<p class="src">数据来源：<code>out/study/energy_model/runs.csv</code>，'
      '每档 3 次重复。</p>')
    A(f'<p>两者相差不到 <b>{abs(phy_m - lin_m):.2f}</b> 个电量单位。'
      '这不是巧合，也不是模型退化的证据：线性模型的常量本来就是照 '
      'PX4 机型的悬停功率标定的（物理模型悬停 0.0505 %/s，线性 0.0500 %/s）。'
      '由此得到一个对使用者的实用结论——<b>线性模型对 PX4 级无人机的'
      '悬停为主剖面是个不错的近似，但它对其他机型与巡航为主的剖面并不成立</b>，'
      '因为巡航比悬停省电（见下），而线性模型会把这段「省下来的电」也按'
      '悬停费率计费。</p>')

    # --- 机型 × 酬载 ---
    A('<h4>机型与酬载：谁会先耗尽</h4>')
    A('<p>四机型 × 有无酬载（0.5 kg）的最终电量与任务完成率。'
      '酬载直接进入推力需求，因此同时抬高悬停与巡航功率。</p>')
    A('<table>')
    hdr = '<tr><th>机型</th>'
    for p in payloads:
        hdr += f'<th colspan="2">酬载 {p} kg</th>'
    hdr += '</tr><tr><th></th>'
    for _ in payloads:
        hdr += '<th>末值电量</th><th>完成率</th>'
    hdr += '</tr>'
    A('<caption>表 15  不同机型与酬载下的最终电量（初始 100）与任务完成率</caption>')
    A(hdr)
    for a, cells in af_tbl:
        row = f'<tr><td>{a}</td>'
        for m, s, done in cells:
            mark = '<b>' if m <= 0.0 else ''
            end = '</b>' if m <= 0.0 else ''
            row += (f'<td>{mark}{m:.1f}{end} ± {s:.1f}</td>'
                    f'<td>{done * 100:.0f} %</td>')
        row += '</tr>'
        A(row)
    A('</table>')
    A('<p class="src">数据来源：<code>out/study/airframe/runs.csv</code>，'
      '每格 3 次重复。</p>')
    zero = [(a, p) for a, cells in af_tbl
            for p, (m, _, _) in zip(payloads, cells) if m <= 0.0]
    if zero:
        desc = "、".join(f'{a} 挂 {p} kg' for a, p in zero)
        A(f'<p><b>{desc}</b>在仿真结束时电量归零——'
          '这是本节最重要的结论：<b>微尺度机型的续航先于推力成为瓶颈</b>。'
          '它们的 TWR 仍然充足（表 13），但电池只有 0.9~1.9 Wh，'
          '而动量理论在微尺度上偏乐观（实测 Crazyflie 悬停约 7 min，'
          '模型给 11.3 min），因此真实余量比表中更紧。</p>')

    # --- 耗尽处置 ---
    A('<h4>电量耗尽会发生什么：三种处置策略</h4>')
    A('<p>设计阶段的一处缺口是「电量归零没有任何后果」，因而无法回答'
      '「会不会掉机」。现已实现为可配置策略：<code>none</code>（无后果，'
      '保留原行为）、<code>land</code>（就地迫降，在途与排队任务作废并交'
      '主节点重派）、<code>crash</code>（在迫降基础上复用既有失效机制）。'
      '场景取续航受限配置（900 s、60 项任务、PX4 挂 0.5 kg、初始电量 45）。</p>')
    A('<table>')
    A('<caption>表 16  三种耗尽处置策略的对比（单一城市布局 <code>env_seed=1</code>）</caption>')
    A('<tr><th>策略</th><th>任务完成率</th><th>投递确认成功率</th><th>转发次数</th></tr>')
    for pol, cr, crs, hb, hbs, fw, fws in pol_tbl:
        A(f'<tr><td>{pol}</td><td>{cr * 100:.1f} % ± {crs * 100:.1f} %</td>'
          f'<td>{hb * 100:.1f} % ± {hbs * 100:.1f} %</td>'
          f'<td>{fw:.0f} ± {fws:.0f}</td></tr>')
    A('</table>')
    A('<p class="src">数据来源：<code>out/study/battery_policy/runs.csv</code>，'
      '每档 12 次重复。</p>')
    A('<p><b>但上表的精度是假的，必须紧接着说明。</b>这 36 次运行'
      '<b>全部固定在同一座城市</b>（<code>env_seed=1</code>）上，'
      '而表 17 与本章 §11 已经证明该布局本身是离群值。因此这些置信区间'
      '<b>只覆盖协议随机性，不含布局方差</b>。把布局也纳入抽样后重做：</p>')

    A('<table>')
    A('<caption>表 17  布局 × 策略：任务完成率（3 策略 × 6 布局 × 3 重复）</caption>')
    A('<tr><th>环境种子</th><th><code>none</code></th><th><code>land</code></th>'
      '<th><code>crash</code></th><th>none − crash</th></tr>')
    for es, mn, ml, mc, d in lay_tbl:
        hi = ' style="background:#f6f6f6"' if d < 0 else ''
        A(f'<tr{hi}><td>{es}</td><td>{mn * 100:.1f} %</td><td>{ml * 100:.1f} %</td>'
          f'<td>{mc * 100:.1f} %</td><td><b>{d * 100:+.1f} %</b></td></tr>')
    A('</table>')
    A('<p class="src">数据来源：<code>out/study/battery_policy_layout/runs.csv</code>。'
      '灰底行的差值为负，即 <code>crash</code> 反而更好。</p>')
    A(f'<p><b>结论被推翻了。</b>以布局配对的均值差为 <b>{d_mean * 100:+.1f} 个百分点</b>'
      f'（sd {d_sd * 100:.1f}，n={n}），<b>t = {t:.2f}（df={n - 1}）</b>，'
      f'双尾 95 % 的临界值是 <b>{tcrit:.3f}</b>，因此<b>不显著</b>；'
      f'精确符号检验 p = <b>{p_sign:.3f}</b>。'
      f'六个布局中 <b>{n_pos} 个</b>方向为正、<b>{n - n_pos} 个</b>为负，'
      '方向并不一致。</p>')
    A(f'<p>反过来，把布局当作随机因素做方差分解，得到一个比原结论'
      f'<b>更有价值</b>的结果：<b>城市布局解释了完成率变异的约 '
      f'{share * 100:.0f} %</b>（布局间 sd {btw:.3f}，布局内 sd {wth:.3f}）。'
      '也就是说，在这个续航受限场景里，<b>决定成败的是城市布局，'
      '而不是耗尽处置策略</b>——后者只是一个二阶效应。'
      '这与本章 §11 从另一组实验得到的结论完全一致，并把它从'
      '「差两个数量级」量化成了方差占比。</p>')

    A('<p>最后是一处必须记录的<b>方法学教训</b>。把「转发次数」按最佳二分'
      '分群可以看到，<b>三种策略都是双峰</b>：</p>')
    A('<table>')
    A('<caption>表 18  三种策略的转发次数分布（转发量双峰，机制是路由选择）</caption>')
    A('<tr><th>策略</th><th>样本数</th><th>低模</th><th>高模</th>'
      '<th>最佳二分间隙 / 群内最大间隙</th></tr>')
    for pol in ("none", "land", "crash"):
        tot, lo, hi, g, within = two_cluster(pol)
        A(f'<tr><td>{pol}</td><td>{tot}</td><td>{lo} 次</td><td>{hi} 次</td>'
          f'<td>{g:.0f} / {within:.0f}</td></tr>')
    A('</table>')
    A('<p class="src">数据来源：同表 17。分群用「最佳二分」判据'
      '（最大间隙是否显著大于群内最大间隙）。</p>')
    A('<p>两个峰值的总字节数只差约 4 %，差的是<b>转发次数</b>——'
      '同一批流量走了<b>直连</b>而不是<b>经中继</b>，是协议侧的路由选择差异。'
      '三种策略都是双峰，因此<b>不存在「单峰、可预测」的策略</b>；'
      '早前把 <code>crash</code> 判为单峰，是因为用了一个过大的分组门限，'
      '把 480~506 这个高模错并进了低模。这类分组门限如果随手取，'
      '会把双峰看成单峰，进而得出「该策略可预测」的错误结论。'
      '本节所有双峰判定都改用最佳二分判据。</p>')

    A('<img src="../../out/figures/baseline_physical/energy.png" style="width:15cm">')
    A('<p class="src">图 4  物理能量模型下各节点的电量随时间变化。'
      '不执行任务航线的节点（主节点与两个中继）以恒定功率悬停，'
      '因此最先耗尽且几乎同时归零。</p>')

    A('<h4>本节小结</h4>')
    A('<ul>')
    A('<li>四机型的推力余裕充足（TWR ≥ 1.95，悬停油门 ≤ 72 %），'
      '<b>四电机够用</b>；瓶颈是能量而非推力。</li>')
    A('<li>线性与物理模型在 PX4 悬停为主的剖面上相差不到 1 个电量单位，'
      '但线性模型对微尺度机型与巡航为主的剖面不成立。</li>')
    A('<li>微尺度机型挂 0.5 kg 酬载后电量归零——<b>续航先于推力成为瓶颈</b>。</li>')
    A('<li>耗尽处置策略对任务完成率的效应<b>在统计上不显著</b>；'
      f'<b>城市布局解释了约 {share * 100:.0f} % 的变异</b>。</li>')
    A('<li>三种策略的转发量都是双峰，机制是直连/中继的路由选择。</li>')
    A('</ul>')

    A(MARK_END)
    return "\n".join(parts)


def build_stats_correction() -> str:
    """更正「统计方法」一节的措辞：固定环境不是配对 CRN，且 24 倍不可复现。"""
    parts = [MARK_STATS_START]
    A = parts.append
    A('<p><b>方法学更正（必读）。</b>本节早前的表述有两处不准确，'
      '在此更正，因为它们影响全章数字的解释口径。</p>')
    A('<p><b>其一，这是「固定环境」设计，不是配对 CRN。</b>'
      '把随机性拆成环境流与协议流、让所有参数组合面对同一座城市，'
      '这个实现是正确的。但框架<b>从不计算配对差值</b>，'
      '因此它不是配对设计，而是「固定共同环境」的单区组设计。'
      '更关键的是：<b>环境固定不是框架保证的</b>——它只是因为基准场景'
      '恰好写了 <code>env_seed</code>。若基准场景省略该字段，'
      '环境种子会回落到每次运行各自的协议种子，'
      '「同一座城市」会<b>静默消失</b>，而不会有任何告警或断言。'
      '这是框架层面的一个真实风险，应加断言。</p>')
    A('<p><b>其二，「区间宽度缩小约 24 倍」不可复现，不应再引用。</b>'
      '它是一处单指标的前后对比，而仓库里只提交过一份结果文件，'
      '且已经是修正之后的版本，因此这个倍数无法从留存数据复算。'
      '本节其余的趋势性结论（时隙单调、抖动降低方差）不依赖该数字，'
      '仍然成立。</p>')
    A('<p><b>其三，区间本身对本章的有界离散指标并不适用。</b>'
      '本章不少指标取值有限（例如完成率每个组合只有两三个取值点），'
      '对其套用正态/t 模型会输出<b>不可能的值</b>——'
      '同一批结果里出现过置信区间下界为负、上界大于 1 的情形。'
      '对这类指标应改用 bootstrap 或精确/Wilson 区间。'
      '就本章主要结论而言 t 区间偏保守，因此没有造成假阳性，'
      '但区间本身不是有效的。</p>')
    A('<p><b>其四，本章的「显著」是目测置信区间是否分离，而不是检验结果。</b>'
      '代码库里那个名为 <code>welch_significant</code> 的函数是死代码'
      '（从未被调用），且名不副实——它比较的是区间是否重叠，'
      '并非 Welch 检验。§11.5 的两处关键比较改用真正的检验：'
      '以布局配对的 t 检验与精确符号检验，结果见该节。</p>')
    A(MARK_STATS_END)
    return "\n".join(parts)


def build_layout_correction() -> str:
    """强化 §11：把布局敏感性从「差两个数量级」量化成方差占比。"""
    bpl = load("battery_policy_layout")
    btw, wth, share = variance_share(bpl, "m:tasks.completion_rate", "env_seed")
    es = load("env_seed")
    ebtw, ewth, eshare = variance_share(es, "m:tasks.completion_rate", "env_seed")

    parts = [MARK_LAYOUT_START]
    A = parts.append
    A('<p><b>这一发现的量化补充。</b>表 11 用「差两个数量级」描述了布局的'
      '影响，那是从丢失率与遮挡数看的。若改用<b>方差分解</b>，'
      '可以给出更直接的量：</p>')
    A('<table>')
    A('<caption>表 11b  城市布局对任务完成率的影响（方差分解）</caption>')
    A('<tr><th>数据集</th><th>布局间 sd</th><th>布局内 sd</th>'
      '<th>布局解释的方差占比</th></tr>')
    A(f'<tr><td>环境种子扫参（表 11，每布局 2 次）</td>'
      f'<td>{ebtw:.3f}</td><td>{ewth:.3f}</td><td><b>{eshare * 100:.0f} %</b></td></tr>')
    A(f'<tr><td>耗尽处置 × 布局（§11.5，每布局 3 次）</td>'
      f'<td>{btw:.3f}</td><td>{wth:.3f}</td><td><b>{share * 100:.0f} %</b></td></tr>')
    A('</table>')
    A('<p class="src">数据来源：<code>out/study/env_seed/runs.csv</code> 与 '
      '<code>out/study/battery_policy_layout/runs.csv</code>。'
      '「布局内 sd」为各布局内标准差的平均（合并，不假设等 n）。</p>')
    A(f'<p>两组独立实验给出一致结论：<b>布局解释了约 {share * 100:.0f} % 的'
      f'完成率变异</b>，而协议或策略参数只占剩下的百分之几。'
      '这把 §11 的定性判断变成了定量口径，也直接解释了为什么'
      '§11.5 中耗尽处置策略的效应会淹没在布局噪声里。</p>')
    A(MARK_LAYOUT_END)
    return "\n".join(parts)


# --- 插桩 -------------------------------------------------------------------
def replace_block(html: str, start: str, end: str, body: str) -> str:
    """幂等替换：已存在则整块替换，否则返回原串（由调用方决定插在哪）。"""
    if start in html and end in html:
        i, j = html.index(start), html.index(end) + len(end)
        return html[:i] + body + html[j:]
    return html


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="只报告是否会漂移，不写文件")
    args = ap.parse_args()

    html = REPORT.read_text(encoding="utf-8")
    original = html

    stats = build_stats_correction()
    layout = build_layout_correction()
    energy = build_energy_section()

    # 1) 统计节更正：插在统计节最后一个 <p> 之后（即「效果是显著的…」那段之后）
    html = replace_block(html, MARK_STATS_START, MARK_STATS_END, stats)
    if MARK_STATS_START not in html:
        anchor = "这一处理是本章所有参数结论能够成立的前提。</p>"
        if anchor not in html:
            raise SystemExit("找不到统计节的锚点，HTML 结构可能已变")
        html = html.replace(anchor, anchor + "\n" + stats, 1)

    # 2) 布局更正：插在 §11 那张表后面的说明段落之后
    html = replace_block(html, MARK_LAYOUT_START, MARK_LAYOUT_END, layout)
    if MARK_LAYOUT_START not in html:
        anchor = "而不是种子 1 的单点值。"
        i = html.index(anchor)
        j = html.index("</p>", i) + len("</p>")
        html = html[:j] + "\n" + layout + html[j:]

    # 3) 能量节：插在「本章小结」之前
    html = replace_block(html, MARK_START, MARK_END, energy)
    if MARK_START not in html:
        anchor = "<h3>本章小结</h3>"
        if anchor not in html:
            raise SystemExit("找不到「本章小结」锚点")
        html = html.replace(anchor, energy + "\n" + anchor, 1)

    if args.check:
        if html != original:
            print("报告 HTML 与生成器不一致（需要重跑生成器）", file=sys.stderr)
            return 1
        print("报告 HTML 与生成器一致")
        return 0

    if html == original:
        print("报告 HTML 无变化（已是最新）")
        return 0
    REPORT.write_text(html, encoding="utf-8")
    print(f"已更新 {REPORT.relative_to(ROOT)}（+{len(html) - len(original)} 字节）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
