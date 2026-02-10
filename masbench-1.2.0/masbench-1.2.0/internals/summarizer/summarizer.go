package summarizer

import (
	"fmt"
	"html/template"
	"masbench/internals/config"
	"masbench/internals/models"
	"masbench/internals/utils"
	"math"
	"os"
	"sort"
	"time"

	"github.com/go-gota/gota/dataframe"
)

const (
	// TIME_TOLERANCE defines the tolerance for considering times as equal (in seconds)
	// TODO: Make this configurable via parameter
	TIME_TOLERANCE = 0.1
)

func GenerateHTMLSummary(benchmarkPaths map[string]string, outputPath string) error {
	report, err := prepareSummaryData(benchmarkPaths)
	if err != nil {
		return fmt.Errorf("failed to prepare summary data: %w", err)
	}

	funcMap := template.FuncMap{
		"add": func(a, b any) float64 {
			var aVal, bVal float64
			switch v := a.(type) {
			case int:
				aVal = float64(v)
			case float64:
				aVal = v
			}
			switch v := b.(type) {
			case int:
				bVal = float64(v)
			case float64:
				bVal = v
			}
			return aVal + bVal
		},
	}

	tmpl, err := template.New("summary").Funcs(funcMap).Parse(summaryTemplate)
	if err != nil {
		return fmt.Errorf("failed to parse template: %w", err)
	}

	file, err := os.Create(outputPath)
	if err != nil {
		return fmt.Errorf("failed to create output file: %w", err)
	}
	defer file.Close()

	if err := tmpl.Execute(file, report); err != nil {
		return fmt.Errorf("failed to execute template: %w", err)
	}

	return nil
}

func prepareSummaryData(benchmarkPaths map[string]string) (SummaryReport, error) {
	dataframes := make(map[string]dataframe.DataFrame)
	benchmarkNames := make([]string, 0, len(benchmarkPaths))

	for name, path := range benchmarkPaths {
		df, err := utils.LoadCSV(path)
		if err != nil {
			return SummaryReport{}, err
		}
		dataframes[name] = df
		benchmarkNames = append(benchmarkNames, name)
	}

	sort.Strings(benchmarkNames)

	allLevels := collectAllLevels(dataframes)

	report := SummaryReport{
		Title:       "Benchmark Summary Report",
		GeneratedAt: time.Now().Format("2006-01-02 15:04:05"),
		Benchmarks:  benchmarkNames,
	}

	report.OverallStats = calculateOverallStats(dataframes, benchmarkNames, allLevels)
	report.LevelSummary = calculateLevelSummary(dataframes, allLevels)
	report.BestByMetric = determineBestByMetric(report.LevelSummary, benchmarkNames)
	report.IndividualStats = calculateIndividualStats(dataframes, benchmarkNames, allLevels, report.LevelSummary)

	return report, nil
}

func collectAllLevels(dataframes map[string]dataframe.DataFrame) []string {
	levelSet := make(map[string]bool)
	for _, df := range dataframes {
		levels := df.Col(models.ColLevelName).Records()
		for _, level := range levels {
			levelSet[level] = true
		}
	}

	levels := make([]string, 0, len(levelSet))
	for level := range levelSet {
		levels = append(levels, level)
	}
	sort.Strings(levels)
	return levels
}

func calculateOverallStats(dataframes map[string]dataframe.DataFrame, benchmarkNames []string, allLevels []string) OverallStats {
	timeout := getDefaultTimeout()
	stats := OverallStats{
		TotalLevels: len(allLevels),
		Timeout:     timeout,
	}

	solvedCounts := make(map[string]int)
	totalTimes := make(map[string]float64)
	avgTimes := make(map[string]float64)
	totalMemory := make(map[string]float64)
	totalStates := make(map[string]float64)
	solvedTimes := make(map[string][]float64)

	for _, name := range benchmarkNames {
		df := dataframes[name]
		dfMap := utils.ToMap(df)

		for _, level := range allLevels {
			data, exists := dfMap[level]
			if !exists {
				totalTimes[name] += float64(timeout)
				continue
			}

			solved := data[models.ColSolved]
			timeVal := utils.GetFloatFromMap(data, models.ColTime)
			memVal := utils.GetFloatFromMap(data, models.ColMemoryAlloc)
			genVal := utils.GetFloatFromMap(data, models.ColGenerated)
			expVal := utils.GetFloatFromMap(data, models.ColExplored)

			if solved == models.SolvedYes {
				solvedCounts[name]++
				totalTimes[name] += timeVal
				solvedTimes[name] = append(solvedTimes[name], timeVal)

				if !math.IsNaN(memVal) && !math.IsInf(memVal, 0) {
					totalMemory[name] += memVal
				}
			} else {
				totalTimes[name] += float64(timeout)
			}

			if !math.IsNaN(genVal) && !math.IsInf(genVal, 0) {
				totalStates[name] += genVal
			}
			if !math.IsNaN(expVal) && !math.IsInf(expVal, 0) {
				totalStates[name] += expVal
			}
		}

		if len(solvedTimes[name]) > 0 {
			sum := 0.0
			for _, t := range solvedTimes[name] {
				sum += t
			}
			avgTimes[name] = sum / float64(len(solvedTimes[name]))
		}
	}

	maxSolved := 0
	for _, count := range solvedCounts {
		if count > maxSolved {
			maxSolved = count
		}
	}

	for _, name := range benchmarkNames {
		if solvedCounts[name] == maxSolved {
			stats.MostLevelsSolved = append(stats.MostLevelsSolved, BenchmarkStat{
				Name:  name,
				Value: fmt.Sprintf("%d levels", maxSolved),
				Extra: fmt.Sprintf("%.2f%% solved", float64(maxSolved)/float64(len(allLevels))*100),
			})
		}
	}

	minTime := math.MaxFloat64
	for _, t := range totalTimes {
		if t < minTime {
			minTime = t
		}
	}

	for _, name := range benchmarkNames {
		if totalTimes[name] == minTime {
			stats.FastestCompletion = append(stats.FastestCompletion, BenchmarkStat{
				Name:  name,
				Value: fmt.Sprintf("%.3fs", minTime),
				Extra: fmt.Sprintf("%d solved", solvedCounts[name]),
			})
		}
	}

	minAvgTime := math.MaxFloat64
	for _, avg := range avgTimes {
		if avg < minAvgTime && avg > 0 {
			minAvgTime = avg
		}
	}

	for _, name := range benchmarkNames {
		if avgTimes[name] == minAvgTime && avgTimes[name] > 0 {
			stats.BestAvgTime = append(stats.BestAvgTime, BenchmarkStat{
				Name:  name,
				Value: fmt.Sprintf("%.3fs avg", minAvgTime),
				Extra: fmt.Sprintf("on %d solved levels", len(solvedTimes[name])),
			})
		}
	}

	minMemory := math.MaxFloat64
	for _, mem := range totalMemory {
		if mem < minMemory {
			minMemory = mem
		}
	}

	for _, name := range benchmarkNames {
		if totalMemory[name] == minMemory {
			stats.LeastMemory = append(stats.LeastMemory, BenchmarkStat{
				Name:  name,
				Value: fmt.Sprintf("%.2f MB", minMemory),
				Extra: "lowest total memory",
			})
		}
	}

	minStates := math.MaxFloat64
	for _, states := range totalStates {
		if states < minStates {
			minStates = states
		}
	}

	for _, name := range benchmarkNames {
		if totalStates[name] == minStates {
			stats.MostEfficient = append(stats.MostEfficient, BenchmarkStat{
				Name:  name,
				Value: fmt.Sprintf("%.0f states", minStates),
				Extra: "fewest explored+generated",
			})
		}
	}

	return stats
}

func calculateLevelSummary(dataframes map[string]dataframe.DataFrame, allLevels []string) []LevelSummary {
	summaries := make([]LevelSummary, 0, len(allLevels))

	for _, level := range allLevels {
		summary := LevelSummary{
			LevelName: level,
		}

		var timeWinners []string
		var actionWinners []string
		minTime := math.MaxFloat64
		minActions := math.MaxFloat64

		for name, df := range dataframes {
			dfMap := utils.ToMap(df)
			data, exists := dfMap[level]
			if !exists {
				summary.NotSolvedBy = append(summary.NotSolvedBy, name)
				continue
			}

			solved := data[models.ColSolved]
			if solved != models.SolvedYes {
				summary.NotSolvedBy = append(summary.NotSolvedBy, name)
				continue
			}

			summary.SolvedBy = append(summary.SolvedBy, name)

			// Track time winners (with tolerance)
			timeVal := utils.GetFloatFromMap(data, models.ColTime)
			if timeVal < minTime-TIME_TOLERANCE {
				// New clear winner
				minTime = timeVal
				timeWinners = []string{name}
			} else if math.Abs(timeVal-minTime) <= TIME_TOLERANCE {
				// Tie - add to winners list
				if !contains(timeWinners, name) {
					timeWinners = append(timeWinners, name)
				}
			}

			// Track action winners (exact comparison)
			actionsVal := utils.GetFloatFromMap(data, models.ColActions)
			if actionsVal < minActions {
				// New clear winner
				minActions = actionsVal
				actionWinners = []string{name}
			} else if actionsVal == minActions {
				// Exact tie - add to winners list
				if !contains(actionWinners, name) {
					actionWinners = append(actionWinners, name)
				}
			}
		}

		// Store time results
		if len(timeWinners) > 0 {
			summary.FastestTime = BenchmarkValue{
				BenchmarkName: timeWinners[0], // Keep first for backward compatibility
				Value:         minTime,
				DisplayValue:  fmt.Sprintf("%.3fs", minTime),
				IsSolved:      true,
			}
			summary.FastestTimeWinners = timeWinners
		} else {
			summary.FastestTime = BenchmarkValue{
				BenchmarkName: "None",
				DisplayValue:  "Not solved",
				IsSolved:      false,
			}
			summary.FastestTimeWinners = []string{}
		}

		// Store action results
		if len(actionWinners) > 0 {
			summary.FewestActions = BenchmarkValue{
				BenchmarkName: actionWinners[0], // Keep first for backward compatibility
				Value:         minActions,
				DisplayValue:  fmt.Sprintf("%.0f", minActions),
				IsSolved:      true,
			}
			summary.FewestActionsWinners = actionWinners
		} else {
			summary.FewestActions = BenchmarkValue{
				BenchmarkName: "None",
				DisplayValue:  "Not solved",
				IsSolved:      false,
			}
			summary.FewestActionsWinners = []string{}
		}

		sort.Strings(summary.SolvedBy)
		sort.Strings(summary.NotSolvedBy)

		summaries = append(summaries, summary)
	}

	return summaries
}

func determineBestByMetric(summaries []LevelSummary, benchmarkNames []string) BestByMetric {
	timeWins := make(map[string]int)
	actionWins := make(map[string]int)

	for _, summary := range summaries {
		// Credit ALL tied winners
		for _, winner := range summary.FastestTimeWinners {
			timeWins[winner]++
		}
		for _, winner := range summary.FewestActionsWinners {
			actionWins[winner]++
		}
	}

	bestTime := findMaxWinner(timeWins, benchmarkNames)
	bestActions := findMaxWinner(actionWins, benchmarkNames)

	return BestByMetric{
		BestTime:    bestTime,
		BestActions: bestActions,
	}
}

func findMaxWinner(wins map[string]int, benchmarkNames []string) string {
	maxWins := 0
	winners := []string{}

	for _, name := range benchmarkNames {
		count := wins[name]
		if count > maxWins {
			maxWins = count
			winners = []string{name}
		} else if count == maxWins && count > 0 {
			winners = append(winners, name)
		}
	}

	if len(winners) == 0 {
		return "None"
	}
	if len(winners) == 1 {
		return fmt.Sprintf("%s (%d levels)", winners[0], maxWins)
	}

	result := ""
	for i, w := range winners {
		if i > 0 {
			result += ", "
		}
		result += w
	}
	return fmt.Sprintf("%s (tie: %d levels each)", result, maxWins)
}

func getDefaultTimeout() int {
	return config.GetConfig().Timeout
}

func calculateIndividualStats(dataframes map[string]dataframe.DataFrame, benchmarkNames []string, allLevels []string, levelSummaries []LevelSummary) []IndividualBenchmarkStats {
	stats := make([]IndividualBenchmarkStats, 0, len(benchmarkNames))

	timeWins := make(map[string]int)
	actionWins := make(map[string]int)
	for _, summary := range levelSummaries {
		// Credit ALL tied winners
		for _, winner := range summary.FastestTimeWinners {
			timeWins[winner]++
		}
		for _, winner := range summary.FewestActionsWinners {
			actionWins[winner]++
		}
	}

	for _, name := range benchmarkNames {
		df := dataframes[name]
		dfMap := utils.ToMap(df)

		individual := IndividualBenchmarkStats{
			Name:        name,
			LevelsTotal: len(allLevels),
			TimeWins:    timeWins[name],
			ActionWins:  actionWins[name],
		}

		solvedCount := 0
		solvedActions := []float64{}
		solvedTimes := []float64{}
		solvedMemory := []float64{}

		for _, level := range allLevels {
			data, exists := dfMap[level]
			if !exists {
				individual.TotalTime += float64(getDefaultTimeout())
				continue
			}

			timeVal := utils.GetFloatFromMap(data, models.ColTime)
			actionsVal := utils.GetFloatFromMap(data, models.ColActions)
			memVal := utils.GetFloatFromMap(data, models.ColMemoryAlloc)
			genVal := utils.GetFloatFromMap(data, models.ColGenerated)
			expVal := utils.GetFloatFromMap(data, models.ColExplored)

			if !math.IsNaN(genVal) && !math.IsInf(genVal, 0) {
				individual.TotalGenerated += genVal
			}
			if !math.IsNaN(expVal) && !math.IsInf(expVal, 0) {
				individual.TotalExplored += expVal
			}

			solved := data[models.ColSolved]
			if solved == models.SolvedYes {
				solvedCount++
				individual.TotalTime += timeVal
				individual.TotalActions += actionsVal
				solvedTimes = append(solvedTimes, timeVal)
				solvedActions = append(solvedActions, actionsVal)

				if !math.IsNaN(memVal) && !math.IsInf(memVal, 0) {
					individual.TotalMemory += memVal
					solvedMemory = append(solvedMemory, memVal)
				}
			} else {
				individual.TotalTime += float64(getDefaultTimeout())
			}
		}

		individual.LevelsSolved = solvedCount
		individual.SolvePercentage = float64(solvedCount) / float64(len(allLevels)) * 100

		if len(solvedTimes) > 0 {
			sum := 0.0
			for _, t := range solvedTimes {
				sum += t
			}
			individual.AvgTime = sum / float64(len(solvedTimes))
		}

		if len(solvedActions) > 0 {
			sum := 0.0
			for _, a := range solvedActions {
				sum += a
			}
			individual.AvgActions = sum / float64(len(solvedActions))
		}

		if len(solvedMemory) > 0 {
			sum := 0.0
			for _, m := range solvedMemory {
				sum += m
			}
			individual.AvgMemory = sum / float64(len(solvedMemory))
		}

		stats = append(stats, individual)
	}

	return stats
}

// contains checks if a string slice contains a specific string
func contains(slice []string, item string) bool {
	for _, s := range slice {
		if s == item {
			return true
		}
	}
	return false
}
