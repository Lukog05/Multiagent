package cmd

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/spf13/cobra"
	"masbench/internals/config"
	"masbench/internals/summarizer"
)

func init() {
	rootCmd.AddCommand(summaryCmd)
}

var summaryCmd = &cobra.Command{
	Use:   "summary <benchmark1> [benchmark2] [benchmark3] ...",
	Short: "Generate an HTML summary report for one or more benchmarks",
	Long: `Generate an interactive HTML summary report showing performance metrics across one or more benchmarks.

This command creates a comprehensive summary showing:
- Which benchmark(s) solved the most levels
- Which benchmark completed in the least time (with timeout for unsolved levels)
- Per-level comparison of fastest time and fewest actions

Examples:
  masbench summary astar-v1
  masbench summary astar-v1 bfs-v1 dijkstra-v1

The generated HTML report provides an easy-to-understand overview of benchmark performance.`,
	Run: func(cmd *cobra.Command, args []string) {
		if len(args) < 1 {
			fmt.Println(colorRed + "Error: You must provide at least one benchmark name." + colorReset)
			os.Exit(1)
		}

		generateSummary(args)
	},
}

func generateSummary(benchmarkNames []string) {
	cfg := config.GetConfig()

	benchmarkPaths := make(map[string]string)
	for _, name := range benchmarkNames {
		path := filepath.Join(cfg.BenchmarkFolder, name, fmt.Sprintf("%s_results.csv", name))
		if _, err := os.Stat(path); os.IsNotExist(err) {
			fmt.Printf(colorRed+"Error: Benchmark result file not found: %s%s\n", name, colorReset)
			os.Exit(1)
		}
		benchmarkPaths[name] = path
	}

	outputDir := filepath.Join(cfg.BenchmarkFolder, "summaries")
	if err := os.MkdirAll(outputDir, 0755); err != nil {
		fmt.Printf(colorRed+"Error creating output directory: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	summaryName := benchmarkNames[0]
	if len(benchmarkNames) > 1 {
		summaryName = "multi_benchmark"
	}
	reportPath := filepath.Join(outputDir, fmt.Sprintf("%s_summary.html", summaryName))

	if err := summarizer.GenerateHTMLSummary(benchmarkPaths, reportPath); err != nil {
		fmt.Printf(colorRed+"Error creating HTML summary: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	fmt.Printf(colorGreen+"Summary completed successfully!%s\n", colorReset)
	fmt.Printf(colorGreen+"HTML Report: %s%s\n", reportPath, colorReset)
	fmt.Printf(colorYellow+"Open the HTML file in your browser to view the interactive report.%s\n", colorReset)
}
