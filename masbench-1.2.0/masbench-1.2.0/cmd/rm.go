package cmd

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"masbench/internals/config"

	"github.com/spf13/cobra"
)

func init() {
	rootCmd.AddCommand(rmCmd)
}

var rmCmd = &cobra.Command{
	Use:   "rm [benchmark_name]",
	Short: "Remove the specified benchmark",
	Long:  `Remove the benchmark by specifying its name, this will delete all the data, log and comparisions related to that benchmark`,
	Args:  cobra.ExactArgs(1),
	Run: func(cmd *cobra.Command, args []string) {
		benchmarkName := args[0]
		rm(benchmarkName)
	},
}

func rm(benchmarkName string) {
	cfg := config.GetConfig()
	benchFolder := cfg.BenchmarkFolder

	dirs, err := os.ReadDir(benchFolder)
	if err != nil {
		fmt.Printf("failed to read directory %s: %s\n", benchFolder, err.Error())
		return
	}

	benchNotFound := true
	for _, entry := range dirs {
		if entry.Name() == benchmarkName && entry.IsDir() {
			benchNotFound = false
			break
		}
	}

	if benchNotFound {
		fmt.Printf("No benchmark called %s was found!!!", benchmarkName)
		return
	}

	err = os.RemoveAll(filepath.Join(benchFolder, benchmarkName))
	if err != nil {
		fmt.Printf("Impossible to delete %s folder, %s", benchFolder, err.Error())
		return
	}

	compFolder := filepath.Join(benchFolder, "comparisons")

	if _, err := os.Stat(compFolder); os.IsNotExist(err) {
		return
	}

	comparisonsDir, err := os.ReadDir(compFolder)
	if err != nil {
		fmt.Printf("failed to read directory %s: %s\n", compFolder, err.Error())
		return
	}

	for _, entry := range comparisonsDir {
		dirName := entry.Name()
		benchmarks := strings.Split(dirName, "vs")

		if len(benchmarks) < 2 {
			continue
		}

		if benchmarks[0] == benchmarkName || benchmarks[1] == benchmarkName {
			os.RemoveAll(filepath.Join(compFolder, dirName))
		}
	}
}
