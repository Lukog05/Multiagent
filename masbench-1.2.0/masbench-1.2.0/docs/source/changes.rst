Changes
=======

This page documents the changes made to the masbench project over time.

Version 1.2.0
-------------

**Improvements:**

* **list** - Now displays benchmark descriptions alongside names. Use ``--name-only`` flag to show only names.
* **run** - Benchmark descriptions are now saved to ``.md`` files for future reference.

Version 1.1.0
-------------

**New Features:**

* **list** - View all benchmarks in your benchmark folder
* **rm** - Remove a benchmark and all related comparisons
* **summary** - Generate HTML summary reports for one or more benchmarks

**Improvements:**

* **compare** - Now generates interactive HTML reports instead of static PNG images. Reports include sortable tables, filterable data, dark mode toggle, and chart export options.

**Bug Fixes:**

* Fixed action count display bug where values of 1000 or more showed only the first digit (e.g., 1000 displayed as 1)

Version 1.0.0 (Initial Release)
--------------------------------

This is the initial release of masbench.

**Features:**

* **init** - Initialize masbench configuration in a repository with interactive setup
* **run** - Execute benchmarks with optional message notes and performance tracking
* **compare** - Compare two benchmark results with visual tables and graphs
* **version** - Display the current version of masbench
