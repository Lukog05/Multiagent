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
	rootCmd.AddCommand(listCmd)
	listCmd.Flags().BoolP("name-only", "n", false, "Show only benchrun names, hide descriptions")
}

var listCmd = &cobra.Command{
	Short: "list all benchmarks",
	Use:   "list",
	Run: func(cmd *cobra.Command, args []string) {
		nameOnly, err := cmd.Flags().GetBool("name-only")
		if err != nil {
			fmt.Println("failed to read flag:", err)
			return
		}
		list(nameOnly)
	},
}

func list(nameOnly bool) {
	cfg := config.GetConfig()
	entries, err := os.ReadDir(cfg.BenchmarkFolder)
	if err != nil {
		fmt.Printf("failed to read directory %s: %s\n", cfg.BenchmarkFolder, err.Error())
	}

	for _, entry := range entries {
		entryName := entry.Name()
		if !entry.IsDir() {
			continue
		}

		// TODO: find a better way rather than hardcoding
		if entryName == "comparisons" {
			continue
		}

		// TODO: find a better way rather than hardcoding
		if entryName == "summaries" {
			continue
		}

		if nameOnly {
			fmt.Println(entry.Name())
			continue
		}

		descriptionFilePath := filepath.Join(cfg.BenchmarkFolder, entryName, entryName+".md")
		descriptionBytes, err := os.ReadFile(descriptionFilePath)

		if err != nil || len(descriptionBytes) == 0 {
			fmt.Println(entryName)
			continue
		}

		description := strings.TrimSpace(string(descriptionBytes))
		fmt.Printf("%s: %s\n", entryName, description)
	}
}
