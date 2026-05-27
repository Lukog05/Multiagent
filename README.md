# Multiagent Search Client

A Python-based AI search client for the DTU course **02285 – AI and Multi-Agent Systems**. The client solves "hospital" planning levels — moving agents and boxes to goal positions — using a range of classical and multi-agent search algorithms.

**Competition score: 51 / 69 levels solved (2026).**

## 🏆 Competition Results (2026)

| Prize | Result |
|---|---|
| 🥉 **Overall Ranking** | **3rd place** |
| ⚡ **Time Ranking** | **1st place** |
| 📦 **Levels Solved** | 51 / 69 |

> Team **Dracarys** — DTU 02285, Spring 2026.

## Repository Layout

```
Multiagent/
├── Warmup/
│   ├── server.jar                      # Level server (Java)
│   ├── levels/                         # Warmup level files (SA*, MA*, MAPF*)
│   ├── temp/                           # Temporary level copies for testing
│   ├── searchclient_python/            # Main Python client
│   │   ├── searchclient/
│   │   │   ├── searchclient.py         # Entry point & cascade orchestrator
│   │   │   ├── state.py                # State representation
│   │   │   ├── action.py               # Action definitions
│   │   │   ├── frontier.py             # BFS / DFS / Best-first frontiers
│   │   │   ├── graphsearch.py          # Generic graph search
│   │   │   ├── heuristic.py            # A*, WA*, Greedy heuristics (Hungarian assignment)
│   │   │   ├── cbs.py                  # CBS, Joint A*, Cooperative A*, PIBT, DFS-CBS
│   │   │   ├── lns.py                  # LNS2 anytime post-processor
│   │   │   ├── ma_planner.py           # Decoupled MA planner + parallel plan compressor
│   │   │   ├── color.py                # Agent/box colour parsing
│   │   │   ├── memory.py               # Memory usage utilities
│   │   │   └── predictability.py       # Predictability-aware heuristic wrapper
│   │   ├── benchmarks/                 # Saved benchmark runs (masbench)
│   │   ├── masbench_config.yml         # Benchmark configuration
│   │   └── pyproject.toml              # Python package metadata
│   ├── searchclient_java/              # Reference Java client
│   ├── run_SA.sh                       # Run all Single-Agent levels
│   ├── run_MA.sh                       # Run all Multi-Agent levels
│   ├── run_MAPF.sh                     # Run all MAPF levels
│   ├── run_SA_gui.sh                   # GUI variant – Single-Agent
│   ├── run_MA_gui.sh                   # GUI variant – Multi-Agent
│   ├── run_MAPF_gui.sh                 # GUI variant – MAPF
│   ├── run_all_levels.sh               # Run every level
│   ├── run_all_levels_gui.sh           # GUI variant – all levels
│   ├── run_suite.sh                    # Full test suite
│   └── debugging.pdf                   # Debugging reference
├── complevels-2026/                    # 69 competition levels (2026)
├── complevels2025/                     # 47 competition levels (2025)
├── masbench-1.2.0/                     # Benchmarking tool (Go)
├── masbench-1.2.0.zip
├── searchclient.zip
└── 2411.06223v2.pdf                    # Reference paper
```

## Prerequisites

- **Java** (for `server.jar`)
- **Python 3.7+** (CPython recommended)
- **psutil** Python package

```bash
pip install psutil
```

- Activate the course conda environment if available:

```bash
conda activate 02285
```

## Running the Client

All commands should be run from the `Warmup/searchclient_python/` directory.

### Single level (GUI)

```bash
# Warmup level
java -jar ../server.jar -l ../levels/SAD1.lvl -c "python3 -m searchclient.searchclient" -g -s 150 -t 180

# 2026 competition level
java -jar ../server.jar -l ../../complevels-2026/AIegean.lvl \
  -c "python3 -m searchclient.searchclient" -g -s 150 -t 180
```

### Competition mode (no GUI, 60 s time budget)

```bash
java -jar ../server.jar -l ../../complevels-2026/<level>.lvl \
  -c "python3 -m searchclient.searchclient --time 60" -t 62
```

### Explicit search strategy

| Flag | Strategy |
|------|----------|
| (none) | Auto-cascade (recommended) |
| `-bfs` | Breadth-First Search |
| `-dfs` | Depth-First Search |
| `-astar` | A* |
| `-wastar [W]` | Weighted A* (default weight 5) |
| `-greedy` | Greedy Best-First |

```bash
java -jar ../server.jar -l ../levels/SAsimple1.lvl \
  -c "python3 -m searchclient.searchclient -astar" -g -s 150 -t 180
```

### Memory limit

```bash
java -jar ../server.jar -l ../levels/SAD1.lvl \
  -c "python3 -m searchclient.searchclient --max-memory 2048" -g -s 150 -t 180
```

Default and recommended maximum: **2 GB**.

## Batch Test Scripts

Run from `Warmup/`:

```bash
bash run_SA.sh              # All Single-Agent levels
bash run_MA.sh              # All Multi-Agent levels
bash run_MAPF.sh            # All MAPF levels
bash run_all_levels.sh      # Every level (SA + MA + MAPF)
bash run_suite.sh           # Full test suite
```

GUI variants (open visualiser window per level):

```bash
bash run_SA_gui.sh
bash run_MA_gui.sh
bash run_MAPF_gui.sh
bash run_all_levels_gui.sh
```

Pass a strategy flag to override the default cascade:

```bash
bash run_SA.sh -astar
```

## Algorithm Overview

### Box levels (Single-Agent / Multi-Agent)

The cascade tries strategies in order; the first success proceeds to LNS2 improvement:

1. **Decoupled greedy planner** (`ma_planner.py`) — assigns each box to a goal and plans pushes one at a time, resolving blockers iteratively. Handles agent-goal placement and displaced-goal recovery. Budget: up to 30 s.
2. **Parallel plan compressor** (`_parallelize_plan` in `ma_planner.py`) — converts the sequential plan into a joint parallel plan using blocker-targeting deadlock recovery and BFS navigation. Significantly reduces action count and makespan.
3. **WA\* cascade** — tried if the decoupled planner fails (skipped if `boxes × agents > 50`):
   - Small levels (≤ 15): `WA*(20) → WA*(10) → WA*(5) → WA*(2)` — lower weights first to avoid wasting time on tight corridors.
   - Larger levels: `WA*(100) → WA*(50) → WA*(20) → WA*(10) → WA*(5)`
4. **Greedy Best-First** — fallback after WA*.
5. **LNS2 post-processing** — WA*(2) restart attempts to shorten the plan (up to 12 s).

### MAPF levels (no boxes)

A cascade is tried in order; the first success proceeds to LNS2 improvement:

| Algorithm | Budget |
|-----------|--------|
| Joint A\* | 20 s (skipped if state space > 6 M) |
| PIBT | 1 s |
| Greedy MAPF | 1 s |
| CBS | 15 s |
| Cooperative A\* | 5 s |
| DFS-CBS | remaining / 2 |
| Greedy Best-First | remainder |

**LNS2 (MAPF):** iterative destroy-and-repair — randomly selects 2–4 agents, replans them under space-time constraints, accepts if makespan decreases.

### Heuristics

- **Manhattan / BFS distance** to nearest goal cell.
- **Hungarian algorithm** (O(n³)) for optimal box-to-goal assignment when multiple boxes exist.
- **Predictability-aware heuristic** — optional wrapper that penalises solutions deviating from expected agent behaviour.

## Benchmarking (masbench)

The `masbench` tool records and compares benchmark runs.

```bash
cd Warmup/searchclient_python
masbench run      # run benchmarks defined in masbench_config.yml
masbench compare  # compare two saved benchmarks
masbench summary  # summarise results
```

Pre-recorded benchmark results are stored in `searchclient_python/benchmarks/`.

## Level Naming Convention

| Prefix | Type |
|--------|------|
| `SA*` | Single-Agent |
| `MA*` | Multi-Agent (with boxes) |
| `MAPF*` | Multi-Agent Path Finding (no boxes) |

## Competition Results (2026)

**51 / 69 levels solved** — competition run with `-t 180` (180 s per level).

| Level | Solved | Actions | Time (s) |
|-------|--------|---------|----------|
| AIegean | ✅ | 75 | 0.038 |
| Agentix | ✅ | 14 | 0.004 |
| AlBarah | ✅ | 22 | 0.007 |
| AlphaMAS | ✅ | 55 | 0.011 |
| Apdo | ❌ | — | — |
| AssertFun | ✅ | 82 | 0.008 |
| BStar | ❌ | — | — |
| BigForty | ✅ | 66 | 0.017 |
| BoxBender | ❌ | — | — |
| CBSquad | ✅ | 1,082 | 0.169 |
| ClauDOom | ❌ | — | — |
| ComMAndos | ✅ | 56 | 0.004 |
| CphAirprt | ✅ | 558 | 0.133 |
| CudBSlvd | ❌ | — | — |
| DASH | ✅ | 204 | 0.015 |
| DayBreak | ✅ | 854 | 0.091 |
| Dolor | ❌ | — | — |
| Dracarys | ✅ | 246 | 0.103 |
| Eighty | ✅ | 72 | 0.008 |
| GHandDirt | ✅ | 55 | 0.013 |
| GroupWon | ❌ | — | — |
| Indianish | ✅ | 456 | 0.096 |
| KUTitans | ✅ | 60 | 0.012 |
| LaMAtes | ✅ | 668 | 0.548 |
| LoopBots | ❌ | — | — |
| LoveLock | ✅ | 443 | 0.047 |
| MAPFlame | ✅ | 120 | 0.012 |
| MASaos | ✅ | 312 | 0.086 |
| MASstroke | ✅ | 95 | 0.023 |
| MAceship | ✅ | 118 | 0.006 |
| MAface | ✅ | 338 | 0.103 |
| MAgic | ✅ | 401 | 0.044 |
| MAmaMASS | ✅ | 52 | 0.003 |
| MArachnid | ✅ | 337 | 155.217 |
| MAuseCat | ✅ | 2,399 | 0.713 |
| MAvis | ✅ | 36 | 0.007 |
| MAze | ✅ | 1,117 | 0.190 |
| MazeRun | ✅ | 348 | 0.028 |
| Minchia | ❌ | — | — |
| Nej | ❌ | — | — |
| NicKiS | ✅ | 416 | 0.031 |
| NineChars | ✅ | 51 | 0.005 |
| OPN | ❌ | — | — |
| OlsenBand | ✅ | 333 | 0.055 |
| PJMAS | ✅ | 257 | 0.022 |
| PinWheel | ✅ | 3,028 | 0.844 |
| Planarchy | ✅ | 281 | 0.054 |
| ProjectD | ❌ | — | — |
| RaedFathi | ✅ | 4 | 0.001 |
| SeisSiete | ✅ | 147 | 0.012 |
| TheDevil | ✅ | 330 | 0.062 |
| Tittling | ✅ | 2,370 | 1.032 |
| TriSplit | ❌ | — | — |
| TriWards | ✅ | 780 | 0.112 |
| TwoPlayer | ✅ | 130 | 0.011 |
| WardRush | ✅ | 443 | 0.066 |
| WeTried | ✅ | 707 | 0.096 |
| amogus | ❌ | — | — |
| brAIn | ❌ | — | — |
| donut | ✅ | 132 | 0.008 |
| duckie | ✅ | 206 | 0.026 |
| escAIpe | ❌ | — | — |
| lilchal | ✅ | 277 | 0.019 |
| logo | ✅ | 54 | 0.007 |
| makeMAga | ✅ | 810 | 2.761 |
| moveMAcat | ✅ | 1,660 | 0.816 |
| slAIve | ❌ | — | — |
| sublevels | ❌ | — | — |
| trauMA | ✅ | 818 | 0.089 |

**Unsolved (18):** Apdo, BStar, BoxBender, ClauDOom, CudBSlvd, Dolor, GroupWon, LoopBots, Minchia, Nej, OPN, ProjectD, TriSplit, amogus, brAIn, escAIpe, slAIve, sublevels
