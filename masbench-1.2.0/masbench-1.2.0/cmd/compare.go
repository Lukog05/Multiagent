package cmd

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/spf13/cobra"
	"masbench/internals/comparator"
	"masbench/internals/config"
	"masbench/internals/utils"
)

func init() {
	rootCmd.AddCommand(compareCmd)
}

var compareCmd = &cobra.Command{
	Use:   "compare <benchmark1> <benchmark2>",
	Short: "Compare two benchmark results and generate an interactive HTML report",
	Long: `Compare two benchmark results and generate an interactive HTML report.

This command creates a comprehensive comparison report showing how benchmark1 
performed relative to benchmark2.

The comparison is benchmark1-centric, meaning all metrics show how benchmark1 
performed compared to benchmark2. To reverse the perspective, swap the order:
  masbench compare benchmark2 benchmark1

Examples:
  masbench compare astar-v1 bfs-v1
  masbench compare optimized-v2 baseline

Note: Both benchmarks must exist in your configured benchmark folder.
The generated HTML report can be opened directly in any web browser.`,
	Run: func(cmd *cobra.Command, args []string) {
		if len(args) < 2 {
			fmt.Println(colorRed + "Error: You must provide two benchmark result files to compare." + colorReset)
			os.Exit(1)
		}

		benchmark1Name := args[0]
		Benchmark2Name := args[1]

		cfg := config.GetConfig()

		benchmark1Path := filepath.Join(cfg.BenchmarkFolder, benchmark1Name, fmt.Sprintf("%s_results.csv", benchmark1Name))
		benchmark2Path := filepath.Join(cfg.BenchmarkFolder, Benchmark2Name, fmt.Sprintf("%s_results.csv", Benchmark2Name))

		if _, err := os.Stat(benchmark1Path); os.IsNotExist(err) {
			fmt.Printf(colorRed+"Error: Benchmark result file not found: %s%s", benchmark1Name, colorReset)
			os.Exit(1)
		}

		if _, err := os.Stat(benchmark2Path); os.IsNotExist(err) {
			fmt.Printf(colorRed+"Error: Benchmark result file not found: %s%s", Benchmark2Name, colorReset)
			os.Exit(1)
		}

		compareResults(benchmark1Path, benchmark2Path, benchmark1Name, Benchmark2Name)
	},
}

func compareResults(benchmark1Path, benchmark2Path, name1, name2 string) {
	df1, err := utils.LoadCSV(benchmark1Path)
	if err != nil {
		fmt.Printf(colorRed+"Error reading benchmark1 CSV: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	df2, err := utils.LoadCSV(benchmark2Path)
	if err != nil {
		fmt.Printf(colorRed+"Error reading benchmark2 CSV: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	cfg := config.GetConfig()
	outputDir := filepath.Join(cfg.BenchmarkFolder, "comparisons", fmt.Sprintf("%svs%s", name1, name2))

	// Create output directory
	if err := os.MkdirAll(outputDir, 0755); err != nil {
		fmt.Printf(colorRed+"Error creating output directory: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	// Generate HTML report
	reportPath := filepath.Join(outputDir, fmt.Sprintf("%svs%s_report.html", name1, name2))
	if err := comparator.GenerateHTMLReport(df1, df2, name1, name2, reportPath); err != nil {
		fmt.Printf(colorRed+"Error creating HTML report: %v%s\n", err, colorReset)
		os.Exit(1)
	}

	fmt.Printf(colorGreen+"Comparison completed successfully!%s\n", colorReset)
	fmt.Printf(colorGreen+"HTML Report: %s%s\n", reportPath, colorReset)
	fmt.Printf(colorYellow+"Open the HTML file in your browser to view the interactive report.%s\n", colorReset)
}
