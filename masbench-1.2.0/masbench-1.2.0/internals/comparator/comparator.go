package comparator

import (
	"fmt"
	"html/template"
	"masbench/internals/models"
	"masbench/internals/utils"
	"os"
	"time"

	"github.com/go-gota/gota/dataframe"
)

func GenerateHTMLReport(df1, df2 dataframe.DataFrame, name1, name2, outputPath string) error {
	report := prepareComparisonData(df1, df2, name1, name2)

	tmpl, err := template.New("report").Parse(reportTemplate)
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

func prepareComparisonData(df1, df2 dataframe.DataFrame, name1, name2 string) ComparisonReport {
	report := ComparisonReport{
		Title:          "Benchmark Comparison Report",
		Benchmark1Name: name1,
		Benchmark2Name: name2,
		GeneratedAt:    time.Now().Format("2006-01-02 15:04:05"),
		MetricNames:    []string{models.ColGenerated, models.ColExplored, models.ColMemoryAlloc, models.ColTime, models.ColActions},
	}

	df2Map := utils.ToMap(df2)

	levelNames1 := df1.Col(models.ColLevelName).Records()

	for i := 0; i < df1.Nrow(); i++ {
		levelName := levelNames1[i]
		levelComp := LevelComparison{
			LevelName: levelName,
		}

		df2Data := df2Map[levelName]

		levelComp.Generated = compareMetric(
			utils.GetFloatFromDF(df1, i, models.ColGenerated),
			utils.GetFloatFromMap(df2Data, models.ColGenerated),
			true,
		)

		levelComp.Explored = compareMetric(
			utils.GetFloatFromDF(df1, i, models.ColExplored),
			utils.GetFloatFromMap(df2Data, models.ColExplored),
			true,
		)

		levelComp.MemoryAlloc = compareMetric(
			utils.GetFloatFromDF(df1, i, models.ColMemoryAlloc),
			utils.GetFloatFromMap(df2Data, models.ColMemoryAlloc),
			true,
		)

		levelComp.Time = compareMetric(
			utils.GetFloatFromDF(df1, i, models.ColTime),
			utils.GetFloatFromMap(df2Data, models.ColTime),
			true,
		)

		levelComp.Actions = compareMetric(
			utils.GetFloatFromDF(df1, i, models.ColActions),
			utils.GetFloatFromMap(df2Data, models.ColActions),
			true,
		)

		solved1 := utils.GetStringFromDF(df1, i, models.ColSolved)
		solved2 := utils.GetStringFromMap(df2Data, models.ColSolved)
		levelComp.Solved = compareSolved(solved1, solved2)

		if solved1 == models.SolvedYes && solved2 == models.SolvedNo {
			levelComp.Generated.Status = "improvement"
			levelComp.Generated.IsImprovement = true
			levelComp.Explored.Status = "improvement"
			levelComp.Explored.IsImprovement = true
			levelComp.MemoryAlloc.Status = "improvement"
			levelComp.MemoryAlloc.IsImprovement = true
			levelComp.Time.Status = "improvement"
			levelComp.Time.IsImprovement = true
			levelComp.Actions.Status = "improvement"
			levelComp.Actions.IsImprovement = true
		} else if solved1 == models.SolvedNo && solved2 == models.SolvedYes {
			levelComp.Generated.Status = "regression"
			levelComp.Generated.IsImprovement = false
			levelComp.Explored.Status = "regression"
			levelComp.Explored.IsImprovement = false
			levelComp.MemoryAlloc.Status = "regression"
			levelComp.MemoryAlloc.IsImprovement = false
			levelComp.Time.Status = "regression"
			levelComp.Time.IsImprovement = false
			levelComp.Actions.Status = "regression"
			levelComp.Actions.IsImprovement = false
		}

		report.Levels = append(report.Levels, levelComp)
	}

	return report
}

func compareMetric(val1, val2 float64, lowerIsBetter bool) MetricComparison {
	diff := val1 - val2
	var diffPct float64
	if val2 != 0 {
		diffPct = (diff / val2) * 100
	}

	var status string
	var isImprovement bool

	if diff == 0 {
		status = "unchanged"
		isImprovement = false
	} else if lowerIsBetter {
		if diff < 0 {
			status = "improvement"
			isImprovement = true
		} else {
			status = "regression"
			isImprovement = false
		}
	}

	return MetricComparison{
		Value1:        val1,
		Value2:        val2,
		Diff:          diff,
		DiffPct:       diffPct,
		Status:        status,
		IsImprovement: isImprovement,
	}
}

func compareSolved(solved1, solved2 string) SolvedComparison {
	changed := solved1 != solved2
	var status string

	if !changed {
		status = "unchanged"
	} else if solved1 == models.SolvedYes && solved2 == models.SolvedNo {
		status = "improvement"
	} else {
		status = "regression"
	}

	return SolvedComparison{
		Solved1: solved1,
		Solved2: solved2,
		Changed: changed,
		Status:  status,
	}
}
