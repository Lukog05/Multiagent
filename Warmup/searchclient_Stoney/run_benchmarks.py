#!/usr/bin/env python3
"""Benchmark script for Exercise 4 - Goal Count Heuristic"""

import subprocess
import re
import sys

LEVELS = [
    "MAPF00.lvl", "MAPF01.lvl", "MAPF02.lvl", "MAPF02C.lvl",
    "MAPF03.lvl", "MAPF03C.lvl", "MAPFslidingpuzzle.lvl", "MAPFreorder2.lvl"
]

ALGORITHMS = ["astar", "greedy"]
TIMEOUT = 180
BASE_PATH = "/Users/hoskuldurthorsteinsson/Documents/GitHub/AI_and_MAS/searchclient"

print("=" * 80)
print("Exercise 4: Goal Count Heuristic Benchmarks")
print("=" * 80)
print()
print("| Level | Algorithm | Expanded | Generated | Actions | Time (s) | Status |")
print("|-------|-----------|----------|-----------|---------|----------|--------|")

for level in LEVELS:
    for algo in ALGORITHMS:
        cmd = [
            "java", "-jar", f"{BASE_PATH}/server.jar",
            "-l", f"{BASE_PATH}/levels/{level}",
            "-c", f"python3 -m searchclient.searchclient -{algo}",
            "-g", "-s", "300", "-t", str(TIMEOUT)
        ]
        
        try:
            result = subprocess.run(
                cmd,
                cwd=f"{BASE_PATH}/searchclient_python",
                capture_output=True,
                text=True,
                timeout=TIMEOUT + 10
            )
            
            output = result.stdout + result.stderr
            
            # Parse output
            expanded_match = re.findall(r'#Expanded:\s*([\d,]+)', output)
            generated_match = re.findall(r'#Generated:\s*([\d,]+)', output)
            actions_match = re.search(r'Actions used:\s*(\d+)', output)
            time_match = re.search(r'Time to solve:\s*([\d.]+)', output)
            solved_match = re.search(r'Level solved:\s*(Yes|No)', output)
            
            expanded = expanded_match[-1].replace(',', '') if expanded_match else "-"
            generated = generated_match[-1].replace(',', '') if generated_match else "-"
            actions = actions_match.group(1) if actions_match else "-"
            time = time_match.group(1) if time_match else "-"
            solved = solved_match.group(1) if solved_match else "ERROR"
            
            print(f"| {level:20} | {algo:7} | {expanded:>8} | {generated:>9} | {actions:>7} | {time:>8} | {solved:6} |")
            
        except subprocess.TimeoutExpired:
            print(f"| {level:20} | {algo:7} | {'':>8} | {'':>9} | {'':>7} | {'>180':>8} | {'TIMEOUT':6} |")
        except Exception as e:
            print(f"| {level:20} | {algo:7} | {'':>8} | {'':>9} | {'':>7} | {'':>8} | {'ERROR':6} |", file=sys.stderr)
            print(f"Error: {e}", file=sys.stderr)

print()
print("=" * 80)
print("Benchmark Complete!")
print("=" * 80)
