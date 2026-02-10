Comparison
==========

This guide explains how to compare benchmark results using masbench's interactive HTML comparison reports to analyze the performance differences between different algorithm implementations or configurations.

Overview
--------

The masbench comparison feature allows you to:

- Compare two benchmark results side by side
- Generate interactive HTML reports with charts and tables
- Visualize performance differences with color-coded metrics
- Sort and filter comparison data
- Export individual charts as PNG images

Basic Comparison
----------------

To compare two benchmark results, use the ``masbench compare`` command:

.. code-block:: bash

   masbench compare benchmark1-name benchmark2-name

For example, to compare an A* implementation against a BFS implementation:

.. code-block:: bash

   masbench compare astar-v1 bfs-v1

.. important::
   Both benchmark results must exist in your benchmark folder. If either benchmark is not found, masbench will display an error message.

Understanding the Comparison
----------------------------

The comparison is **benchmark1-centric**, meaning all metrics show how ``benchmark1`` performed relative to ``benchmark2``:

- **Green** indicates ``benchmark1`` performed better
- **Red** indicates ``benchmark1`` performed worse  
- **Gray** indicates no difference

To reverse the comparison perspective, simply swap the benchmark order:

.. code-block:: bash

   masbench compare benchmark2-name benchmark1-name

Prerequisites
-------------

Before running comparisons, ensure you have:

1. At least two completed benchmark runs
2. Valid CSV result files for both benchmarks
3. The benchmark names match the folder names in your benchmark directory

Generated Output
----------------

When you run a comparison, masbench creates a new folder with an interactive HTML report:

.. code-block:: text

   benchmark-results/
   └── comparisons/
       └── benchmark1vsbenchmark2/
           └── benchmark1vsbenchmark2_report.html

Opening the Report
~~~~~~~~~~~~~~~~~~

Simply open the HTML file in your web browser:

.. code-block:: bash

   # Linux
   xdg-open benchmark-results/comparisons/benchmark1vsbenchmark2/benchmark1vsbenchmark2_report.html
   
   # macOS
   open benchmark-results/comparisons/benchmark1vsbenchmark2/benchmark1vsbenchmark2_report.html
   
   # Windows
   start benchmark-results/comparisons/benchmark1vsbenchmark2/benchmark1vsbenchmark2_report.html

Report Features
---------------

Interactive Comparison Table
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The main comparison table shows all metrics for each level:

- **Level Name**: The benchmark level identifier
- **Generated**: Number of states generated (lower is better)
- **Explored**: Number of states explored (lower is better)
- **Memory**: Memory allocated in MB (lower is better)
- **Time**: Execution time in seconds (lower is better)
- **Actions**: Number of actions in solution (lower is better)
- **Solved**: Whether the level was solved

Each metric displays:
- Current values: ``value1 vs value2``
- Difference: Shows how much better/worse benchmark1 is
- Color coding: Green (better), Red (worse), Gray (same)

**Table Features:**

- **Search**: Filter levels by name using the search box
- **Filter**: Show only improvements or regressions
- **Sort**: Click column headers to sort by any metric
- **Responsive**: Automatically adapts to your screen size

Interactive Charts
~~~~~~~~~~~~~~~~~~

Charts are organized in tabs for each metric:

- Generated States Comparison
- Explored States Comparison
- Memory Allocation Comparison
- Time Comparison
- Actions Comparison

**Chart Features:**

- Interactive tooltips showing exact values
- Zoom and pan capabilities
- Export individual charts as PNG images
- Side-by-side bar comparison
- Responsive sizing

Interpreting Results
--------------------

Understanding Metrics
~~~~~~~~~~~~~~~~~~~~~

All metrics use "lower is better" logic:

- **Generated/Explored**: Fewer states means more efficient search
- **Memory**: Lower memory usage means better resource efficiency
- **Time**: Faster execution is better
- **Actions**: Fewer actions means shorter solution path
- **Solved**: "Yes" is better than "No"

Reading Differences
~~~~~~~~~~~~~~~~~~~

Differences show how much benchmark1 differs from benchmark2:

.. code-block:: text

   100 vs 120
   -20 (-16.7%)
   
This means:
- benchmark1 value: 100
- benchmark2 value: 120  
- benchmark1 is 20 units lower (better)
- This is a 16.7% improvement

Example Workflow
----------------

1. Run two benchmarks:

   .. code-block:: bash

      masbench benchmark --name astar-optimized
      masbench benchmark --name astar-baseline

2. Compare them:

   .. code-block:: bash

      masbench compare astar-optimized astar-baseline

3. Open the generated HTML report in your browser

4. If you want to see the opposite perspective:

   .. code-block:: bash

      masbench compare astar-baseline astar-optimized

.. seealso::
   - For running benchmarks, see the :doc:`running_benchmarks` guide
   - For initial setup, see the :doc:`getting_started` guide
