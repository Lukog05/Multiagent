package cmd

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"
)

var rootCmd = &cobra.Command{
	Use:   "masbench",
	Short: "A benchmark tool for multi-agent systems.",
	Long:  `masbench is a CLI tool to benchmark multi-agent systems.`,
}

func Execute() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
