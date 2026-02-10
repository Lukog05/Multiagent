package comparator

type ComparisonReport struct {
	Title          string
	Benchmark1Name string
	Benchmark2Name string
	GeneratedAt    string
	Levels         []LevelComparison
	MetricNames    []string
}

type LevelComparison struct {
	LevelName   string
	Generated   MetricComparison
	Explored    MetricComparison
	MemoryAlloc MetricComparison
	Time        MetricComparison
	Actions     MetricComparison
	Solved      SolvedComparison
}

type MetricComparison struct {
	Value1        float64
	Value2        float64
	Diff          float64
	DiffPct       float64
	Status        string // "improvement", "regression", "unchanged"
	IsImprovement bool
}

type SolvedComparison struct {
	Solved1 string
	Solved2 string
	Changed bool
	Status  string // "improved", "regressed", "unchanged"
}

type ChartData struct {
	Labels     []string
	Dataset1   []float64
	Dataset2   []float64
	MetricName string
}
