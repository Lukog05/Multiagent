package summarizer

const summaryTemplate = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{{.Title}}</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body {
            color: #000000;
            background-color: #f9fafb;
        }
        .dark {
            background-color: #111827 !important;
        }
        .dark .bg-white { background-color: #1f2937 !important; }
        .dark .bg-gray-50 { background-color: #111827 !important; }
        .dark .border-gray-200 { border-color: #4b5563 !important; }
        .dark .border-gray-300 { border-color: #4b5563 !important; }
        .dark .border-gray-700 { border-color: #4b5563 !important; }
        .dark thead { background-color: #1f2937 !important; }
        .dark tbody { background-color: #1f2937 !important; }
        .dark .divide-gray-200 > * { border-color: #4b5563 !important; }
        .dark .bg-blue-50 { background-color: #1e3a8a !important; }
        .dark .bg-purple-50 { background-color: #581c87 !important; }
        .dark .bg-green-50 { background-color: #064e3b !important; }
        .dark .bg-orange-50 { background-color: #7c2d12 !important; }
        .dark .bg-yellow-50 { background-color: #78350f !important; }
        .dark .bg-gray-800 { background-color: #1f2937 !important; }
        .sortable:hover {
            cursor: pointer;
            background-color: #f3f4f6;
        }
        .dark .sortable:hover {
            background-color: #4b5563;
        }
        .winner {
            background: linear-gradient(135deg, #fef3c7 0%, #fde68a 100%);
            border-left: 4px solid #f59e0b;
        }
        .dark .winner {
            background: linear-gradient(135deg, #78350f 0%, #92400e 100%);
            border-left: 4px solid #fbbf24;
        }
        .solved {
            background-color: #dcfce7;
        }
        .dark .solved {
            background-color: #064e3b;
        }
        .not-solved {
            background-color: #fee2e2;
        }
        .dark .not-solved {
            background-color: #7f1d1d;
        }
        .badge {
            display: inline-block;
            padding: 0.25rem 0.75rem;
            border-radius: 9999px;
            font-size: 0.75rem;
            font-weight: 600;
        }
        .badge-success {
            background-color: #dcfce7;
            color: #166534;
        }
        .dark .badge-success {
            background-color: #064e3b;
            color: #4ade80;
        }
        .badge-danger {
            background-color: #fee2e2;
            color: #991b1b;
        }
        .dark .badge-danger {
            background-color: #7f1d1d;
            color: #f87171;
        }
        .metric-card {
            transition: transform 0.2s;
        }
        .metric-card:hover {
            transform: translateY(-2px);
        }
    </style>
</head>
<body class="bg-gray-50 transition-colors duration-200">
    <!-- Header -->
    <div class="bg-white shadow-sm border-b border-gray-200">
        <div class="max-w-7xl mx-auto px-4 py-6">
            <div class="flex justify-between items-center">
                <div>
                    <h1 class="text-3xl font-bold text-gray-900">📊 Benchmark Summary Report</h1>
                    <div class="mt-2 flex items-center gap-2 text-sm text-gray-600 flex-wrap">
                        {{range $i, $name := .Benchmarks}}
                            {{if $i}}<span class="text-gray-400">•</span>{{end}}
                            <span class="font-semibold text-blue-600">{{$name}}</span>
                        {{end}}
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

    <!-- Overall Statistics -->
    <div class="max-w-7xl mx-auto px-4 py-6">
        <div class="flex justify-between items-center mb-4">
            <h2 class="text-2xl font-bold text-gray-900">📈 Overall Statistics</h2>
            <div class="text-sm text-gray-600">
                <span class="font-medium">Total Levels:</span> 
                <span class="text-lg font-bold text-gray-900">{{.OverallStats.TotalLevels}}</span>
            </div>
        </div>
        <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
            <!-- Most Levels Solved -->
            <div class="bg-white rounded-lg shadow border border-gray-200 p-6 metric-card winner">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-gray-900">🏆 Most Levels Solved</p>
                    <div class="text-4xl">👑</div>
                </div>
                {{range .OverallStats.MostLevelsSolved}}
                <div class="mb-2">
                    <p class="text-xl font-bold text-gray-900">{{.Name}}</p>
                    <p class="text-sm text-gray-600">{{.Value}}</p>
                    <p class="text-xs text-gray-500">{{.Extra}}</p>
                </div>
                {{end}}
            </div>

            <!-- Fastest Completion -->
            <div class="bg-white rounded-lg shadow border border-gray-200 p-6 metric-card winner">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-gray-900">⚡ Fastest Total Time</p>
                    <div class="text-4xl">🚀</div>
                </div>
                {{range .OverallStats.FastestCompletion}}
                <div class="mb-2">
                    <p class="text-xl font-bold text-gray-900">{{.Name}}</p>
                    <p class="text-sm text-gray-600">{{.Value}}</p>
                    <p class="text-xs text-gray-500">{{.Extra}}</p>
                </div>
                {{end}}
            </div>

            <!-- Best Average Time -->
            {{if .OverallStats.BestAvgTime}}
            <div class="bg-white rounded-lg shadow border border-gray-200 p-6 metric-card winner">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-gray-900">⏱️ Best Avg Time</p>
                    <div class="text-4xl">📊</div>
                </div>
                {{range .OverallStats.BestAvgTime}}
                <div class="mb-2">
                    <p class="text-xl font-bold text-gray-900">{{.Name}}</p>
                    <p class="text-sm text-gray-600">{{.Value}}</p>
                    <p class="text-xs text-gray-500">{{.Extra}}</p>
                </div>
                {{end}}
            </div>
            {{end}}

            <!-- Least Memory -->
            {{if .OverallStats.LeastMemory}}
            <div class="bg-white rounded-lg shadow border border-gray-200 p-6 metric-card winner">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-gray-900">💾 Memory Efficient</p>
                    <div class="text-4xl">🧠</div>
                </div>
                {{range .OverallStats.LeastMemory}}
                <div class="mb-2">
                    <p class="text-xl font-bold text-gray-900">{{.Name}}</p>
                    <p class="text-sm text-gray-600">{{.Value}}</p>
                    <p class="text-xs text-gray-500">{{.Extra}}</p>
                </div>
                {{end}}
            </div>
            {{end}}

            <!-- Best by Time -->
            <div class="bg-blue-50 rounded-lg shadow border border-blue-200 p-6 metric-card">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-blue-900">🏅 Best by Time</p>
                    <div class="text-3xl">⏱️</div>
                </div>
                <p class="text-sm text-blue-700 mb-2">Most levels won</p>
                <p class="text-lg font-bold text-blue-900">{{.BestByMetric.BestTime}}</p>
            </div>

            <!-- Best by Actions -->
            <div class="bg-green-50 rounded-lg shadow border border-green-200 p-6 metric-card">
                <div class="flex items-center justify-between mb-3">
                    <p class="text-sm font-medium text-green-900">🎖️ Best by Actions</p>
                    <div class="text-3xl">🎯</div>
                </div>
                <p class="text-sm text-green-700 mb-2">Most levels won</p>
                <p class="text-lg font-bold text-green-900">{{.BestByMetric.BestActions}}</p>
            </div>
        </div>
    </div>

    <!-- Info Banner -->
    <div class="max-w-7xl mx-auto px-4 py-4">
        <div class="bg-yellow-50 dark:bg-yellow-900 border-l-4 border-yellow-500 p-4 rounded-lg">
            <div class="flex items-start">
                <div class="flex-shrink-0">
                    <svg class="h-5 w-5 text-yellow-500" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clip-rule="evenodd" />
                    </svg>
                </div>
                <div class="ml-3">
                    <p class="text-sm text-yellow-700 dark:text-yellow-200">
                        <strong>Note:</strong> For unsolved levels, a timeout of 300s is used in total time calculations.
                        The table below shows which benchmark performed best on each individual level.
                    </p>
                </div>
            </div>
        </div>
    </div>

    <!-- Individual Benchmark Statistics -->
    <div class="max-w-7xl mx-auto px-4 py-8">
        <div class="bg-white rounded-lg shadow border border-gray-200">
            <div class="p-6 border-b border-gray-200">
                <div class="flex justify-between items-center">
                    <h2 class="text-2xl font-bold text-gray-900">🔍 Individual Benchmark Details</h2>
                    <select id="benchmarkSelector" class="px-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 font-medium" onchange="selectBenchmark(this.value)">
                        {{range $i, $stat := .IndividualStats}}
                        <option value="{{$i}}" {{if eq $i 0}}selected{{end}}>{{$stat.Name}}</option>
                        {{end}}
                    </select>
                </div>
            </div>

            {{range $i, $stat := .IndividualStats}}
            <div id="benchmark-{{$i}}" class="benchmark-detail {{if ne $i 0}}hidden{{end}}">
                <div class="p-6">
                    <h3 class="text-xl font-bold text-gray-900 mb-6">📊 {{$stat.Name}} Performance</h3>
                    
                    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
                        <!-- Levels Solved -->
                        <div class="bg-blue-50 dark:bg-blue-900 rounded-lg p-4 border border-blue-200 dark:border-blue-700">
                            <p class="text-xs font-medium text-blue-700 dark:text-blue-300 uppercase">Levels Solved</p>
                            <p class="text-2xl font-bold text-blue-900 dark:text-blue-100 mt-1">{{$stat.LevelsSolved}} / {{$stat.LevelsTotal}}</p>
                            <p class="text-sm text-blue-600 dark:text-blue-300 mt-1">{{printf "%.1f%%" $stat.SolvePercentage}}</p>
                        </div>

                        <!-- Total Time -->
                        <div class="bg-purple-50 dark:bg-purple-900 rounded-lg p-4 border border-purple-200 dark:border-purple-700">
                            <p class="text-xs font-medium text-purple-700 dark:text-purple-300 uppercase">Total Time</p>
                            <p class="text-2xl font-bold text-purple-900 dark:text-purple-100 mt-1">{{printf "%.2fs" $stat.TotalTime}}</p>
                            <p class="text-sm text-purple-600 dark:text-purple-300 mt-1">{{if gt $stat.LevelsSolved 0}}incl. timeouts{{else}}all timeouts{{end}}</p>
                        </div>

                        <!-- Average Time -->
                        <div class="bg-green-50 dark:bg-green-900 rounded-lg p-4 border border-green-200 dark:border-green-700">
                            <p class="text-xs font-medium text-green-700 dark:text-green-300 uppercase">Average Time</p>
                            <p class="text-2xl font-bold text-green-900 dark:text-green-100 mt-1">{{printf "%.3fs" $stat.AvgTime}}</p>
                            <p class="text-sm text-green-600 dark:text-green-300 mt-1">{{if gt $stat.LevelsSolved 0}}on solved levels{{else}}N/A{{end}}</p>
                        </div>

                        <!-- Total Actions -->
                        <div class="bg-orange-50 dark:bg-orange-900 rounded-lg p-4 border border-orange-200 dark:border-orange-700">
                            <p class="text-xs font-medium text-orange-700 dark:text-orange-300 uppercase">Total Actions</p>
                            <p class="text-2xl font-bold text-orange-900 dark:text-orange-100 mt-1">{{printf "%.0f" $stat.TotalActions}}</p>
                            <p class="text-sm text-orange-600 dark:text-orange-300 mt-1">Avg: {{printf "%.1f" $stat.AvgActions}}</p>
                        </div>
                    </div>

                    <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
                        <!-- Memory Stats -->
                        <div class="bg-white dark:bg-gray-800 rounded-lg p-4 border border-gray-200 dark:border-gray-700">
                            <h4 class="text-sm font-semibold text-gray-900 dark:text-gray-100 mb-3">💾 Memory Usage</h4>
                            <div class="space-y-2">
                                <div class="flex justify-between">
                                    <span class="text-sm text-gray-600 dark:text-gray-300">Total Memory:</span>
                                    <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{printf "%.2f MB" $stat.TotalMemory}}</span>
                                </div>
                                <div class="flex justify-between">
                                    <span class="text-sm text-gray-600 dark:text-gray-300">Average Memory:</span>
                                    <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{printf "%.2f MB" $stat.AvgMemory}}</span>
                                </div>
                            </div>
                        </div>

                        <!-- State Space Stats -->
                        <div class="bg-white dark:bg-gray-800 rounded-lg p-4 border border-gray-200 dark:border-gray-700">
                            <h4 class="text-sm font-semibold text-gray-900 dark:text-gray-100 mb-3">🔬 State Space Exploration</h4>
                            <div class="space-y-2">
                                <div class="flex justify-between">
                                    <span class="text-sm text-gray-600 dark:text-gray-300">Generated States:</span>
                                    <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{printf "%.0f" $stat.TotalGenerated}}</span>
                                </div>
                                <div class="flex justify-between">
                                    <span class="text-sm text-gray-600 dark:text-gray-300">Explored States:</span>
                                    <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{printf "%.0f" $stat.TotalExplored}}</span>
                                </div>
                                <div class="flex justify-between pt-2 border-t border-gray-300 dark:border-gray-600">
                                    <span class="text-sm text-gray-600 dark:text-gray-300">Total States:</span>
                                    <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{printf "%.0f" (add $stat.TotalGenerated $stat.TotalExplored)}}</span>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- Wins Stats - Full Width -->
                    <div class="bg-white dark:bg-gray-800 rounded-lg p-4 border border-gray-200 dark:border-gray-700 mt-4">
                        <h4 class="text-sm font-semibold text-gray-900 dark:text-gray-100 mb-3">🏆 Level Wins</h4>
                        <div class="grid grid-cols-1 md:grid-cols-3 gap-4">
                            <div class="flex justify-between">
                                <span class="text-sm text-gray-600 dark:text-gray-300">⚡ Fastest Time Wins:</span>
                                <span class="text-sm font-bold text-blue-600 dark:text-blue-400">{{$stat.TimeWins}} levels</span>
                            </div>
                            <div class="flex justify-between">
                                <span class="text-sm text-gray-600 dark:text-gray-300">🎯 Fewest Actions Wins:</span>
                                <span class="text-sm font-bold text-green-600 dark:text-green-400">{{$stat.ActionWins}} levels</span>
                            </div>
                            <div class="flex justify-between">
                                <span class="text-sm text-gray-600 dark:text-gray-300">Total Wins:</span>
                                <span class="text-sm font-bold text-gray-900 dark:text-gray-100">{{add $stat.TimeWins $stat.ActionWins}} wins</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            {{end}}
        </div>
    </div>

    <!-- Level-by-Level Summary Table -->
    <div class="max-w-7xl mx-auto px-4 py-8">
        <div class="bg-white rounded-lg shadow border border-gray-200">
            <div class="p-6 border-b border-gray-200">
                <div class="flex justify-between items-center">
                    <h2 class="text-2xl font-bold text-gray-900">🎮 Level-by-Level Performance</h2>
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
                            <option value="solved">Solved by All</option>
                            <option value="partial">Partially Solved</option>
                            <option value="unsolved">Unsolved by All</option>
                        </select>
                    </div>
                </div>
            </div>
            <div class="overflow-x-auto">
                <table id="summaryTable" class="min-w-full divide-y divide-gray-200">
                    <thead class="bg-gray-50">
                        <tr>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(0)">
                                Level Name ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(1)">
                                ⚡ Fastest Time ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider sortable" onclick="sortTable(2)">
                                🎯 Fewest Actions ↕
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                                ✅ Solved By
                            </th>
                            <th class="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                                ❌ Not Solved By
                            </th>
                        </tr>
                    </thead>
                    <tbody class="bg-white divide-y divide-gray-200">
                        {{range .LevelSummary}}
                        <tr data-solved-count="{{len .SolvedBy}}" data-total-count="{{add (len .SolvedBy) (len .NotSolvedBy)}}">
                            <td class="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                                {{.LevelName}}
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                                {{if gt (len .FastestTimeWinners) 0}}
                                    <div class="font-semibold text-blue-600">{{.FastestTime.DisplayValue}}</div>
                                    <div class="flex flex-wrap gap-1 mt-1">
                                        {{range .FastestTimeWinners}}
                                            <span class="badge badge-success">{{.}}</span>
                                        {{end}}
                                    </div>
                                {{else}}
                                    <span class="text-red-600">Not solved</span>
                                {{end}}
                            </td>
                            <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                                {{if gt (len .FewestActionsWinners) 0}}
                                    <div class="font-semibold text-green-600">{{.FewestActions.DisplayValue}}</div>
                                    <div class="flex flex-wrap gap-1 mt-1">
                                        {{range .FewestActionsWinners}}
                                            <span class="badge badge-success">{{.}}</span>
                                        {{end}}
                                    </div>
                                {{else}}
                                    <span class="text-red-600">Not solved</span>
                                {{end}}
                            </td>
                            <td class="px-6 py-4 text-sm text-gray-900">
                                {{if .SolvedBy}}
                                    <div class="flex flex-wrap gap-1">
                                        {{range .SolvedBy}}
                                            <span class="badge badge-success">{{.}}</span>
                                        {{end}}
                                    </div>
                                {{else}}
                                    <span class="text-gray-400">None</span>
                                {{end}}
                            </td>
                            <td class="px-6 py-4 text-sm text-gray-900">
                                {{if .NotSolvedBy}}
                                    <div class="flex flex-wrap gap-1">
                                        {{range .NotSolvedBy}}
                                            <span class="badge badge-danger">{{.}}</span>
                                        {{end}}
                                    </div>
                                {{else}}
                                    <span class="text-gray-400">None</span>
                                {{end}}
                            </td>
                        </tr>
                        {{end}}
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <!-- Footer -->
    <div class="max-w-7xl mx-auto px-4 py-8 text-center text-sm text-gray-500">
        <p>Generated by masbench summary command</p>
        <p class="mt-1">Tailwind CSS v3.4.0</p>
    </div>

    <script>
        // Benchmark selector
        function selectBenchmark(index) {
            document.querySelectorAll('.benchmark-detail').forEach(el => el.classList.add('hidden'));
            document.getElementById('benchmark-' + index).classList.remove('hidden');
        }

        // Dark mode toggle
        function toggleDarkMode() {
            document.body.classList.toggle('dark');
        }

        // Table filtering
        function filterTable() {
            const searchValue = document.getElementById('searchInput').value.toLowerCase();
            const filterValue = document.getElementById('filterSelect').value;
            const table = document.getElementById('summaryTable');
            const rows = table.getElementsByTagName('tbody')[0].getElementsByTagName('tr');

            for (let row of rows) {
                const levelName = row.cells[0].textContent.toLowerCase();
                const solvedCount = parseInt(row.getAttribute('data-solved-count'));
                const totalCount = parseInt(row.getAttribute('data-total-count'));
                
                let showRow = true;
                
                if (searchValue && !levelName.includes(searchValue)) {
                    showRow = false;
                }
                
                if (filterValue === 'solved' && solvedCount !== totalCount) {
                    showRow = false;
                } else if (filterValue === 'partial' && (solvedCount === 0 || solvedCount === totalCount)) {
                    showRow = false;
                } else if (filterValue === 'unsolved' && solvedCount !== 0) {
                    showRow = false;
                }
                
                row.style.display = showRow ? '' : 'none';
            }
        }

        // Table sorting
        let sortDirection = {};
        function sortTable(columnIndex) {
            const table = document.getElementById('summaryTable');
            const tbody = table.getElementsByTagName('tbody')[0];
            const rows = Array.from(tbody.getElementsByTagName('tr'));
            
            const direction = sortDirection[columnIndex] === 'asc' ? 'desc' : 'asc';
            sortDirection[columnIndex] = direction;
            
            rows.sort((a, b) => {
                let aValue = a.cells[columnIndex].textContent.trim();
                let bValue = b.cells[columnIndex].textContent.trim();
                
                const aNum = parseFloat(aValue);
                const bNum = parseFloat(bValue);
                
                if (!isNaN(aNum) && !isNaN(bNum)) {
                    return direction === 'asc' ? aNum - bNum : bNum - aNum;
                }
                
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
