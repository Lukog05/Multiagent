package cmd

import (
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"

	"masbench/internals/config"
	"masbench/internals/parsers"

	"github.com/spf13/cobra"
)

var message string

func init() {
	rootCmd.AddCommand(runCmd)
	runCmd.Flags().StringVarP(&message, "message", "m", "", "Add a note to the run")
}

var runCmd = &cobra.Command{
	Use:   "run [benchmark-name]",
	Short: "Run a benchmark with masbench",
	Long:  `This command executes a benchmark using masbench. It requires a configuration file to be present in the current directory.`,
	Args:  cobra.ExactArgs(1),
	Run: func(cmd *cobra.Command, args []string) {
		benchmarkName := args[0]
		fmt.Printf("Running benchmark: %s\n", benchmarkName)
		runBenchmark(benchmarkName, message)
	},
}

func runBenchmark(name string, message string) {
	cfg := config.GetConfig()

	// Create benchmark folder if it does not exist
	if _, err := os.Stat(cfg.BenchmarkFolder); os.IsNotExist(err) {
		if err := os.MkdirAll(cfg.BenchmarkFolder, os.ModePerm); err != nil {
			fmt.Printf("\033[31mError creating benchmark folder: %v\033[0m\n", err)
			return
		}
	}

	benchmarkPath := filepath.Join(cfg.BenchmarkFolder, name)

	// If the benchmark folder for that name already exists, print an error and exit.
	if _, err := os.Stat(benchmarkPath); !os.IsNotExist(err) {
		fmt.Printf("\033[31mError: Benchmark with name '%s' already exists. Please remove it before running a new one.\033[0m\n", name)
		return
	}

	logServerPath := filepath.Join(benchmarkPath, "logs", fmt.Sprintf("%s_server.zip", name))
	logClientPath := filepath.Join(benchmarkPath, "logs", fmt.Sprintf("%s_client.clog", name))

	// Create the logs directory
	logDir := filepath.Dir(logServerPath)
	if err := os.MkdirAll(logDir, os.ModePerm); err != nil {
		fmt.Printf("\033[31mError creating log directory: %v\033[0m\n", err)
		return
	}

	// Create the client log file
	logFile, err := os.Create(logClientPath)
	if err != nil {
		fmt.Printf("\033[31mError creating client log file: %v\033[0m\n", err)
		return
	}
	defer logFile.Close()

	cmd := exec.Command("java", "-jar", cfg.ServerPath,
		"-l", cfg.LevelsDir,
		"-o", logServerPath,
		"-c", cfg.ClientCommand,
		"-t", fmt.Sprintf("%d", cfg.Timeout),
	)

	multiWriter := io.MultiWriter(os.Stdout, logFile)
	cmd.Stdout = multiWriter
	cmd.Stderr = multiWriter

	err = cmd.Run()
	if err != nil {
		fmt.Printf("\033[31mError running benchmark: %v\033[0m\n", err)
		os.RemoveAll(benchmarkPath)
		return
	}

	fmt.Println("\033[32mBenchmark run completed successfully.\033[0m")

	// After the benchmark, parse the client log to CSV
	csvOutputPath := filepath.Join(benchmarkPath, fmt.Sprintf("%s_results.csv", name))
	err = parsers.ParseLogToCSV(logClientPath, csvOutputPath)
	if err != nil {
		fmt.Printf("\033[31mError parsing log to CSV: %v\033[0m\n", err)
		return
	}

	descriptionFilePath := filepath.Join(benchmarkPath, name+".md")
	err = os.WriteFile(descriptionFilePath, []byte(message+"\n"), 0644)
	if err != nil {
		fmt.Printf("\033[31mError! Couldn't write in %s \n %v\033[0m\n", descriptionFilePath, err)
	}

	fmt.Printf("\033[32mResults successfully written to %s\033[0m\n", csvOutputPath)
}
