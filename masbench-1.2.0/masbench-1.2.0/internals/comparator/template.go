package comparator

// TODO: Consider using the embed library to load into the .exe / bin file
// the html template so that we can have in the code base an html file
// with the template instead of writing it here in go

const reportTemplate = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{{.Title}}</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/html2canvas@1.4.1/dist/html2canvas.min.js"></script>
    <style>
        body {
            color: #000000;
            background-color: #f9fafb;
        }
        .dark {
            background-color: #111827 !important;
            color: #ffffff !important;
        }
        .dark * {
            color: #ffffff;
        }
        .dark .text-gray-900 {
            color: #ffffff !important;
        }
        .dark .text-gray-600 {
            color: #d1d5db !important;
        }
        .dark .text-gray-500 {
            color: #9ca3af !important;
        }
        .dark table {
            color: #ffffff;
        }
        .dark .bg-white {
            background-color: #1f2937 !important;
        }
        .dark .bg-gray-50 {
            background-color: #111827 !important;
        }
        .dark .border-gray-200 {
            border-color: #4b5563 !important;
        }
        .sortable:hover {
            cursor: pointer;
            background-color: #f3f4f6;
        }
        .dark .sortable:hover {
            background-color: #4b5563;
        }
        .dark thead {
            background-color: #1f2937 !important;
        }
        .improvement {
            background-color: #dcfce7;
            color: #000000;
        }
        .regression {
            background-color: #fee2e2;
            color: #000000;
        }
        .unchanged {
            background-color: #f3f4f6;
            color: #000000;
        }
        .dark .improvement {
            background-color: #064e3b;
            color: #ffffff !important;
        }
        .dark .regression {
            background-color: #7f1d1d;
            color: #ffffff !important;
        }
        .dark .unchanged {
            background-color: #374151;
            color: #ffffff !important;
        }
        .dark tbody {
            background-color: #1f2937 !important;
        }
        .dark .divide-gray-200 > * {
            border-color: #4b5563 !important;
        }
        .dark .text-green-600 {
            color: #4ade80 !important;
        }
        .dark .text-red-600 {
            color: #f87171 !important;
        }
        .dark .text-blue-600 {
            color: #60a5fa !important;
        }
        .dark .text-orange-600 {
            color: #fb923c !important;
        }
    </style>
</head>
<body class="bg-gray-50 transition-colors duration-200">
    <!-- Header -->
    <div class="bg-white shadow-sm border-b border-gray-200">
        <div class="max-w-7xl mx-auto px-4 py-6">
            <div class="flex justify-between items-center">
                <div>
                    <h1 class="text-3xl font-bold text-gray-900">📊 Benchmark Comparison Report</h1>
                    <div class="mt-2 flex items-center gap-4 text-sm text-gray-600">
                        <span class="font-semibold text-blue-600">{{.Benchmark1Name}}</span>
                        <span class="text-gray-400">vs</span>
                        <span class="font-semibold text-orange-600">{{.Benchmark2Name}}</span>
                        <span class="text-gray-400">•</span>
                        <span>{{.GeneratedAt}}</span>
                    </div>
                </div>
                <button onclick="toggleDarkMode()" class="px-4 py-2 bg-gray-800 text-white rounded-lg hover:bg-gray-700 transition">
                    🌙 Dark Mode
                </button>
            </div>
        </div>
    </div>

    <!-- Info Banner -->
    <div class="max-w-7xl mx-auto px-4 py-6">
        <div class="bg-blue-50 dark:bg-blue-900 border-l-4 border-blue-500 p-4 rounded-lg">
            <div class="flex items-start">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-blue-500" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-blue-700 dark:text-blue-200">
                        This comparison shows how <strong>{{.Benchmark1Name}}</strong> performed compared to <strong>{{.Benchmark2Name}}</strong>.
                        Green indicates <strong>{{.Benchmark1Name}}</strong> is better, Red indicates worse.
                        To reverse the comparison, run: <code class="bg-blue-100 dark:bg-blue-800 px-2 py-1 rounded">masbench compare {{.Benchmark2Name}} {{.Benchmark1Name}}</code>
                    </p>
                </div>
            </div>
        </div>
    </div>

    <!-- Comparison Table -->
    <div class="max-w-7xl mx-auto px-4 py-8">
        <div class="bg-white rounded-lg shadow border border-gray-200">
            <div class="p-6 border-b border-gray-200">
                <div class="flex justify-between items-center">
                    <h2 class="text-2xl font-bold text-gray-900">Detailed Comparison</h2>
                    <div class="flex gap-4">
                        <input 
                            type="text" 
                            id="searchInput" 
                            placeholder="🔍 Search levels..." 
                            class="px-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
                            onkeyup="filterTable()"
                        >
                        <select id="filterSelect" class="px-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500" onchange="filterTable()">
                            <option value="all">All Levels</option>
                            <option value="improvement">Improvements Only</option>
                            <option value="regression">Regressions Only</option>
                        </select>
                    </div>
                </div>
            </div>
            <div class="overflow-x-auto">
                <table id="comparisonTable" class="min-w-full divide-y divide-gray-200">
                    <thead class="bg-gray-50">
                        <tr>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(0)">
                                Level Name ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(1)">
                                Generated ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(2)">
                                Explored ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(3)">
                                Memory ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(4)">
                                Time ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(5)">
                                Actions ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(6)">
                                Solved ↕
                            </th>
                        </tr>
                    </thead>
                    <tbody class="bg-white divide-y divide-gray-200">
                        {{range .Levels}}
                        <tr data-status="{{.Generated.Status}}">
                            <td class="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                                {{.LevelName}}
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.Generated.Status}}">
                                <div class="font-semibold">{{printf "%.0f vs %.0f" .Generated.Value1 .Generated.Value2}}</div>
                                <div class="text-xs {{if .Generated.IsImprovement}}text-green-600{{else if eq .Generated.Status "regression"}}text-red-600{{else}}text-gray-500{{end}}">
                                    {{printf "%.0f (%.1f%%)" .Generated.Diff .Generated.DiffPct}}
                                </div>
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.Explored.Status}}">
                                <div class="font-semibold">{{printf "%.0f vs %.0f" .Explored.Value1 .Explored.Value2}}</div>
                                <div class="text-xs {{if .Explored.IsImprovement}}text-green-600{{else if eq .Explored.Status "regression"}}text-red-600{{else}}text-gray-500{{end}}">
                                    {{printf "%.0f (%.1f%%)" .Explored.Diff .Explored.DiffPct}}
                                </div>
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.MemoryAlloc.Status}}">
                                <div class="font-semibold">{{printf "%.2f vs %.2f" .MemoryAlloc.Value1 .MemoryAlloc.Value2}}</div>
                                <div class="text-xs {{if .MemoryAlloc.IsImprovement}}text-green-600{{else if eq .MemoryAlloc.Status "regression"}}text-red-600{{else}}text-gray-500{{end}}">
                                    {{printf "%.2f (%.1f%%)" .MemoryAlloc.Diff .MemoryAlloc.DiffPct}}
                                </div>
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.Time.Status}}">
                                <div class="font-semibold">{{printf "%.3f vs %.3f" .Time.Value1 .Time.Value2}}</div>
                                <div class="text-xs {{if .Time.IsImprovement}}text-green-600{{else if eq .Time.Status "regression"}}text-red-600{{else}}text-gray-500{{end}}">
                                    {{printf "%.3f (%.1f%%)" .Time.Diff .Time.DiffPct}}
                                </div>
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.Actions.Status}}">
                                <div class="font-semibold">{{printf "%.0f vs %.0f" .Actions.Value1 .Actions.Value2}}</div>
                                <div class="text-xs {{if .Actions.IsImprovement}}text-green-600{{else if eq .Actions.Status "regression"}}text-red-600{{else}}text-gray-500{{end}}">
                                    {{printf "%.0f (%.1f%%)" .Actions.Diff .Actions.DiffPct}}
                                </div>
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900 {{.Solved.Status}}">
                                <div class="font-semibold">{{.Solved.Solved1}} vs {{.Solved.Solved2}}</div>
                            </td>
                        </tr>
                        {{end}}
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <!-- Charts Section -->
    <div class="max-w-7xl mx-auto px-4 py-8">
        <div class="bg-white rounded-lg shadow border border-gray-200">
            <div class="p-6">
                <h2 class="text-2xl font-bold text-gray-900 mb-6">Interactive Charts</h2>
                
                <!-- Tabs -->
                <div class="flex border-b border-gray-200 mb-6">
                    <button class="tab-button px-4 py-2 font-medium border-b-2 border-blue-500 text-blue-600" onclick="showChart('generated')">
                        Generated
                    </button>
                    <button class="tab-button px-4 py-2 font-medium text-gray-500 hover:text-gray-700" onclick="showChart('explored')">
                        Explored
                    </button>
                    <button class="tab-button px-4 py-2 font-medium text-gray-500 hover:text-gray-700" onclick="showChart('memory')">
                        Memory
                    </button>
                    <button class="tab-button px-4 py-2 font-medium text-gray-500 hover:text-gray-700" onclick="showChart('time')">
                        Time
                    </button>
                    <button class="tab-button px-4 py-2 font-medium text-gray-500 hover:text-gray-700" onclick="showChart('actions')">
                        Actions
                    </button>
                </div>

                <!-- Chart Containers -->
                <div id="chart-generated" class="chart-container">
                    <div class="flex justify-between items-center mb-4">
                        <h3 class="text-xl font-semibold">Generated States Comparison</h3>
                        <button onclick="exportChart('generatedChart')" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition">
                            📥 Export PNG
                        </button>
                    </div>
                    <canvas id="generatedChart"></canvas>
                </div>

                <div id="chart-explored" class="chart-container hidden">
                    <div class="flex justify-between items-center mb-4">
                        <h3 class="text-xl font-semibold">Explored States Comparison</h3>
                        <button onclick="exportChart('exploredChart')" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition">
                            📥 Export PNG
                        </button>
                    </div>
                    <canvas id="exploredChart"></canvas>
                </div>

                <div id="chart-memory" class="chart-container hidden">
                    <div class="flex justify-between items-center mb-4">
                        <h3 class="text-xl font-semibold">Memory Allocation Comparison</h3>
                        <button onclick="exportChart('memoryChart')" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition">
                            📥 Export PNG
                        </button>
                    </div>
                    <canvas id="memoryChart"></canvas>
                </div>

                <div id="chart-time" class="chart-container hidden">
                    <div class="flex justify-between items-center mb-4">
                        <h3 class="text-xl font-semibold">Time Comparison</h3>
                        <button onclick="exportChart('timeChart')" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition">
                            📥 Export PNG
                        </button>
                    </div>
                    <canvas id="timeChart"></canvas>
                </div>

                <div id="chart-actions" class="chart-container hidden">
                    <div class="flex justify-between items-center mb-4">
                        <h3 class="text-xl font-semibold">Actions Comparison</h3>
                        <button onclick="exportChart('actionsChart')" class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition">
                            📥 Export PNG
                        </button>
                    </div>
                    <canvas id="actionsChart"></canvas>
                </div>
            </div>
        </div>
    </div>

    <!-- Footer -->
    <div class="max-w-7xl mx-auto px-4 py-8 text-center text-sm text-gray-500">
        <p>Generated by masbench compare command</p>
        <p class="mt-1">Chart.js v4.4.0 | Tailwind CSS v3.4.0</p>
    </div>

    <script>
        // Chart data from Go template
        const chartData = {
            labels: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}"{{$v.LevelName}}"{{end}}],
            generated1: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Generated.Value1}}{{end}}],
            generated2: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Generated.Value2}}{{end}}],
            explored1: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Explored.Value1}}{{end}}],
            explored2: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Explored.Value2}}{{end}}],
            memory1: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.MemoryAlloc.Value1}}{{end}}],
            memory2: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.MemoryAlloc.Value2}}{{end}}],
            time1: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Time.Value1}}{{end}}],
            time2: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Time.Value2}}{{end}}],
            actions1: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Actions.Value1}}{{end}}],
            actions2: [{{range $i, $v := .Levels}}{{if $i}}, {{end}}{{$v.Actions.Value2}}{{end}}],
        };

        const benchmark1Name = "{{.Benchmark1Name}}";
        const benchmark2Name = "{{.Benchmark2Name}}";

        // Chart configurations
        const chartConfig = {
            type: 'bar',
            options: {
                responsive: true,
                maintainAspectRatio: true,
                aspectRatio: 2.5,
                plugins: {
                    legend: {
                        display: true,
                        position: 'top',
                    },
                    tooltip: {
                        mode: 'index',
                        intersect: false,
                    }
                },
                scales: {
                    x: {
                        ticks: {
                            autoSkip: false,
                            maxRotation: 90,
                            minRotation: 45,
                            font: {
                                size: 10
                            }
                        }
                    },
                    y: {
                        beginAtZero: true
                    }
                }
            }
        };

        // Initialize charts
        let charts = {};

        function createChart(id, label, data1, data2) {
            const ctx = document.getElementById(id).getContext('2d');
            charts[id] = new Chart(ctx, {
                ...chartConfig,
                data: {
                    labels: chartData.labels,
                    datasets: [
                        {
                            label: benchmark1Name,
                            data: data1,
                            backgroundColor: 'rgba(59, 130, 246, 0.5)',
                            borderColor: 'rgb(59, 130, 246)',
                            borderWidth: 1
                        },
                        {
                            label: benchmark2Name,
                            data: data2,
                            backgroundColor: 'rgba(249, 115, 22, 0.5)',
                            borderColor: 'rgb(249, 115, 22)',
                            borderWidth: 1
                        }
                    ]
                }
            });
        }

        // Create all charts
        createChart('generatedChart', 'Generated', chartData.generated1, chartData.generated2);
        createChart('exploredChart', 'Explored', chartData.explored1, chartData.explored2);
        createChart('memoryChart', 'Memory', chartData.memory1, chartData.memory2);
        createChart('timeChart', 'Time', chartData.time1, chartData.time2);
        createChart('actionsChart', 'Actions', chartData.actions1, chartData.actions2);

        // Show/hide charts
        function showChart(metric) {
            // Hide all chart containers
            document.querySelectorAll('.chart-container').forEach(el => el.classList.add('hidden'));
            // Show selected chart
            document.getElementById('chart-' + metric).classList.remove('hidden');
            
            // Update tab styles
            document.querySelectorAll('.tab-button').forEach(btn => {
                btn.classList.remove('border-blue-500', 'text-blue-600');
                btn.classList.add('text-gray-500');
            });
            event.target.classList.add('border-blue-500', 'text-blue-600');
            event.target.classList.remove('text-gray-500');
        }

        // Export chart as PNG
        function exportChart(chartId) {
            const canvas = document.getElementById(chartId);
            const url = canvas.toDataURL('image/png');
            const link = document.createElement('a');
            link.download = chartId + '.png';
            link.href = url;
            link.click();
        }

        // Dark mode toggle
        function toggleDarkMode() {
            document.body.classList.toggle('dark');
            const isDark = document.body.classList.contains('dark');
            
            // Update all charts
            Object.values(charts).forEach(chart => {
                chart.options.plugins.legend.labels.color = isDark ? '#f3f4f6' : '#000';
                chart.options.scales.x.ticks.color = isDark ? '#f3f4f6' : '#666';
                chart.options.scales.y.ticks.color = isDark ? '#f3f4f6' : '#666';
                chart.options.scales.x.grid.color = isDark ? '#4b5563' : '#e5e7eb';
                chart.options.scales.y.grid.color = isDark ? '#4b5563' : '#e5e7eb';
                chart.update();
            });
        }

        // Table filtering
        function filterTable() {
            const searchValue = document.getElementById('searchInput').value.toLowerCase();
            const filterValue = document.getElementById('filterSelect').value;
            const table = document.getElementById('comparisonTable');
            const rows = table.getElementsByTagName('tbody')[0].getElementsByTagName('tr');

            for (let row of rows) {
                const levelName = row.cells[0].textContent.toLowerCase();
                const status = row.getAttribute('data-status');
                
                let showRow = true;
                
                // Search filter
                if (searchValue && !levelName.includes(searchValue)) {
                    showRow = false;
                }
                
                // Status filter
                if (filterValue !== 'all' && status !== filterValue) {
                    showRow = false;
                }
                
                row.style.display = showRow ? '' : 'none';
            }
        }

        // Table sorting
        let sortDirection = {};
        function sortTable(columnIndex) {
            const table = document.getElementById('comparisonTable');
            const tbody = table.getElementsByTagName('tbody')[0];
            const rows = Array.from(tbody.getElementsByTagName('tr'));
            
            const direction = sortDirection[columnIndex] === 'asc' ? 'desc' : 'asc';
            sortDirection[columnIndex] = direction;
            
            rows.sort((a, b) => {
                let aValue = a.cells[columnIndex].textContent.trim();
                let bValue = b.cells[columnIndex].textContent.trim();
                
                // Try to parse as number
                const aNum = parseFloat(aValue);
                const bNum = parseFloat(bValue);
                
                if (!isNaN(aNum) && !isNaN(bNum)) {
                    return direction === 'asc' ? aNum - bNum : bNum - aNum;
                }
                
                // String comparison
                return direction === 'asc' ? 
                    aValue.localeCompare(bValue) : 
                    bValue.localeCompare(aValue);
            });
            
            rows.forEach(row => tbody.appendChild(row));
        }
    </script>
</body>
</html>
`
