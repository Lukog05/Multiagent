Getting Started
===============

This guide will walk you through setting up masbench in your project and configuring it for your specific needs.

Installation
------------

To install masbench:

1. Download the appropriate zip file for your OS and processor from the `releases page <https://github.com/MarlonBando/masbench/releases/tag/v1.1.0>`_
2. Unzip the downloaded file
3. Add the extracted folder to your system PATH

That's it! You can now use the ``masbench`` command from anywhere in your terminal.

Initializing masbench
---------------------

To get started with masbench, you need to initialize it in your project. This is done using the ``masbench init`` command.

.. important::
   It is **highly recommended** to run this command from your project's root directory. This ensures that the ``masbench_config.yml`` file is created in the root folder, making it easier to manage and locate.

.. code-block:: bash

   # Navigate to your project root
   cd /path/to/your/project
   
   # Initialize masbench
   masbench init

When you run this command, masbench will ask if you want to continue with the initialization in the current directory. After confirming, you'll have two options:

1. **Guided initialization**: Follow an interactive dialog to configure all settings
2. **Default configuration**: Create a default configuration file that you can manually edit later

Guided Initialization
~~~~~~~~~~~~~~~~~~~~~

If you choose the guided initialization, masbench will prompt you for each configuration setting:

- **Server path**: Path to your server executable (.jar file)
- **Levels directory**: Directory containing the levels you want to benchmark
- **Benchmark folder**: Where benchmark results will be stored
- **Client command**: The command to start your client

Default Configuration
~~~~~~~~~~~~~~~~~~~~~

If you choose to skip the guided setup, a default ``masbench_config.yml`` file will be created with placeholder values that you can edit manually.

Configuration Settings
----------------------

After initialization, you'll have a ``masbench_config.yml`` file in your project root. Here's what each setting does:

ServerPath
~~~~~~~~~~

This is the path to your server executable file. The server must be a ``.jar`` file.

**Example:**

.. code-block:: yaml

   ServerPath: "/home/user/my-project/server.jar"

LevelsDir
~~~~~~~~~

This specifies the directory containing the levels you want to benchmark.

.. tip::
   Create a separate folder containing only the levels you want to benchmark. This reduces benchmark time by avoiding unnecessary files.

**Example:**

.. code-block:: yaml

   LevelsDir: "path/to/your/benchmark-levels"

BenchmarkFolder
~~~~~~~~~~~~~~~

This is where masbench will store all benchmark results and logs.
If the folder does not exist, masbench will create it automatically.

**Examples:**

.. code-block:: yaml

    BenchmarkFolder: "path/to/your/benchmark-results"

ClientCommand
~~~~~~~~~~~~~

This is the command used to start your client. This command will be passed to the server using the ``-c`` option.

.. note::
   - Do **not** include ``java -jar server.jar`` in this command
   - Do **not** include the level path - masbench handles this automatically
   - Include any flags or options your client needs

**Examples:**
Let's say you run your level with the following commands:

.. code-block:: shell

   java -jar server/server.jar -l server/levels/SAsoko3_16.lvl -c "python -m project.src.searchclient -greedy --max-memory 1024" -g -s 150 -t 500

What masbench need is just the client command that is the one after the ``-c`` option:

.. code-block:: yaml

   ClientCommand: "python -m project.src.searchclient -greedy --max-memory 1024"

Timeout
~~~~~~~

This sets the maximum time (in seconds) that each level benchmark can run before being terminated.

**Example:**

.. code-block:: yaml

   Timeout: 300  # 5 minutes timeout

Sample Configuration
--------------------

Here's a complete example of a ``masbench_config.yml`` file:

.. code-block:: yaml

   ServerPath: "/home/user/my-ai-project/server.jar"
   LevelsDir: "/home/user/my-ai-project/test-levels"
   BenchmarkFolder: "benchmark-results"
   ClientCommand: "python -m src.searchclient --algorithm astar --max-memory 2048"
   Timeout: 300

Next Steps
----------

Once your configuration is set up:

1. Verify your server and client work correctly
2. Place your test levels in the specified levels directory
3. Run your first benchmark with ``masbench run``
4. Check the results in your benchmark folder

.. seealso::
   - For running benchmarks, see the :doc:`running_benchmarks` guide
   - For comparing results, see the :doc:`comparison` guide