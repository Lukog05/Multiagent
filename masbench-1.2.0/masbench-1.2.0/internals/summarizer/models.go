package summarizer

type SummaryReport struct {
	Title           string
	GeneratedAt     string
	Benchmarks      []string
	OverallStats    OverallStats
	LevelSummary    []LevelSummary
	BestByMetric    BestByMetric
	IndividualStats []IndividualBenchmarkStats
}

type OverallStats struct {
	TotalLevels       int
	Timeout           int
	MostLevelsSolved  []BenchmarkStat
	FastestCompletion []BenchmarkStat
	BestAvgTime       []BenchmarkStat
	LeastMemory       []BenchmarkStat
	MostEfficient     []BenchmarkStat
}

type BenchmarkStat struct {
	Name  string
	Value string
	Extra string
}

type LevelSummary struct {
	LevelName            string
	FastestTime          BenchmarkValue
	FastestTimeWinners   []string
	FewestActions        BenchmarkValue
	FewestActionsWinners []string
	SolvedBy             []string
	NotSolvedBy          []string
}

type BenchmarkValue struct {
	BenchmarkName string
	Value         float64
	DisplayValue  string
	IsSolved      bool
}

type BestByMetric struct {
	BestTime    string
	BestActions string
}

type IndividualBenchmarkStats struct {
	Name            string
	LevelsSolved    int
	LevelsTotal     int
	SolvePercentage float64
	TotalTime       float64
	AvgTime         float64
	TotalActions    float64
	AvgActions      float64
	TotalMemory     float64
	AvgMemory       float64
	TotalGenerated  float64
	TotalExplored   float64
	TimeWins        int
	ActionWins      int
}
