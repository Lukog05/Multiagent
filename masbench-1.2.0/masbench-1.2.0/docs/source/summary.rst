Summary Reports
===============

This guide explains how to generate summary reports that analyze performance across one or more benchmarks.

Basic Usage
-----------

Single Benchmark Summary
~~~~~~~~~~~~~~~~~~~~~~~~

To analyze one benchmark:

.. code-block:: bash

   masbench summary benchmark-name

This generates a report showing:

- Total levels solved
- Total time taken
- Per-level breakdown of performance

Multiple Benchmark Summary
~~~~~~~~~~~~~~~~~~~~~~~~~~

To analyze multiple benchmarks together:

.. code-block:: bash

   masbench summary benchmark1 benchmark2 benchmark3

For example:

.. code-block:: bash

   masbench summary astar-v1 bfs-v1 dijkstra-v1

This creates a report showing:

- Which benchmark solved the most levels
- Which benchmark finished fastest (timeout applies to unsolved levels)
- For each level: which benchmark had the fastest time and fewest actions

Generated Output
----------------

When you run a summary, masbench creates an HTML report:

.. code-block:: text

   benchmark-results/
   └── summaries/
       ├── benchmark1_summary.html          (single benchmark)
       └── multi_benchmark_summary.html     (multiple benchmarks)

Opening the Report
~~~~~~~~~~~~~~~~~~

Open the HTML file in your browser:

.. code-block:: bash

   # Linux
   xdg-open benchmark-results/summaries/benchmark1_summary.html
   
   # macOS
   open benchmark-results/summaries/benchmark1_summary.html
   
   # Windows
   start benchmark-results/summaries/benchmark1_summary.html

Report Features
---------------

Overall Performance
~~~~~~~~~~~~~~~~~~~

The report header shows:

- **Levels Solved**: Total count for each benchmark
- **Total Time**: Cumulative time (unsolved levels count as timeout)
- **Winner**: Benchmark with best overall performance

Level-by-Level Analysis
~~~~~~~~~~~~~~~~~~~~~~~

Each level shows:

- **Fastest Time**: Which benchmark solved it quickest
- **Fewest Actions**: Which benchmark used the shortest solution

Example Workflow
----------------

1. Run multiple benchmarks:

   .. code-block:: bash

      masbench run astar-heuristic-1
      masbench run astar-heuristic-2
      masbench run astar-heuristic-3

2. Generate a summary:

   .. code-block:: bash

      masbench summary astar-heuristic-1 astar-heuristic-2 astar-heuristic-3

3. Open the HTML report

4. Analyze results:
   - Check which heuristic solved the most levels
   - Identify levels where each heuristic excels
   - Look for patterns in performance differences
   - Use the data to choose the best approach

5. For detailed comparison between two specific benchmarks:

   .. code-block:: bash

      masbench compare astar-heuristic-1 astar-heuristic-2

Use Cases
---------

Heuristic Tuning
~~~~~~~~~~~~~~~~

Compare variations of the same algorithm:

.. code-block:: bash

   masbench summary baseline heuristic-v1 heuristic-v2 heuristic-v3

Find which parameter values work best.

Single Benchmark Review
~~~~~~~~~~~~~~~~~~~~~~~~

Analyze one benchmark's performance:

.. code-block:: bash

   masbench summary my-algorithm

Review which levels were solved, which failed, and where time was spent.

.. seealso::
   - For comparing two benchmarks, see the :doc:`comparison` guide
   - For running benchmarks, see the :doc:`running_benchmarks` guide
   - For initial setup, see the :doc:`getting_started` guide
