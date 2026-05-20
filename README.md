# Multiagent Search Client

A Python-based AI search client for the DTU course **02285 – AI and Multi-Agent Systems**. The client solves "hospital" planning levels — moving agents and boxes to goal positions — using a range of classical and multi-agent search algorithms.

**Competition score: 30 / 47 levels solved.**

## Repository Layout

```
Multiagent/
├── Warmup/
│   ├── server.jar                      # Level server (Java)
│   ├── levels/                         # All level files (SA*, MA*, MAPF*)
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
│   │   │   ├── ma_planner.py           # Decoupled greedy multi-agent box planner
│   │   │   └── predictability.py       # Predictability-aware heuristic wrapper
│   │   ├── benchmarks/                 # Saved benchmark runs (masbench)
│   │   └── masbench_config.yml         # Benchmark configuration
│   ├── searchclient_java/              # Reference Java client
│   ├── run_SA.sh                       # Run all Single-Agent levels
│   ├── run_MA.sh                       # Run all Multi-Agent levels
│   ├── run_MAPF.sh                     # Run all MAPF levels
│   ├── run_*_gui.sh                    # GUI variants of the above
│   └── run_suite.sh                    # Run the full test suite
├── complevels/                         # 47 competition levels
├── masbench-1.2.0/                     # Benchmarking tool (Go)
└── masbench-1.2.0.zip
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

All commands should be run from the `Warmup/` directory.

### Single level (GUI)

```bash
java -jar server.jar -l levels/SAD1.lvl -c "python -m searchclient.searchclient" -g -s 150 -t 180
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
java -jar server.jar -l levels/SAsimple1.lvl \
  -c "python -m searchclient.searchclient -astar" -g -s 150 -t 180
```

### Memory limit

```bash
java -jar server.jar -l levels/SAD1.lvl \
  -c "python -m searchclient.searchclient --max-memory 2048" -g -s 150 -t 180
```

Default and recommended maximum: **2 GB**.

## Batch Test Scripts

Run from `Warmup/`:

```bash
bash run_SA.sh          # All Single-Agent levels
bash run_MA.sh          # All Multi-Agent levels
bash run_MAPF.sh        # All MAPF levels
bash run_suite.sh       # Full suite
```

Pass a strategy flag to override the default cascade:

```bash
bash run_SA.sh -astar
```

## Algorithm Overview

### Box levels (Single-Agent / Multi-Agent)

The cascade tries strategies in order; the first success proceeds to LNS2 improvement:

1. **Decoupled greedy planner** (`ma_planner.py`) — assigns each box to a goal and plans pushes one at a time, resolving blockers iteratively. Handles agent-goal placement and displaced-goal recovery. Budget: up to 140 s for complex levels.
2. **WA\* cascade** — tried if the decoupled planner fails (skipped if `boxes × agents > 50`):
   - Small levels (≤ 15): `WA*(20) → WA*(10) → WA*(5) → WA*(2)` — lower weights first to avoid wasting time on tight corridors.
   - Larger levels: `WA*(100) → WA*(50) → WA*(20) → WA*(10) → WA*(5)`
3. **Greedy Best-First** — fallback after WA*.
4. **Second decoupled pass** — uses any remaining time.
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
