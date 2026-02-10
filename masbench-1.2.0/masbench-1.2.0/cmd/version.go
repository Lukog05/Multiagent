package cmd

import (
	"fmt"
	"os"
	"strings"

	"github.com/spf13/cobra"
)

func init() {
	rootCmd.AddCommand(versionCmd)
}

var versionCmd = &cobra.Command{
	Use:   "version",
	Short: "Print the version number of masbench",
	Long:  `All software has versions. This is masbench's`,
	Run: func(cmd *cobra.Command, args []string) {
		version := getVersion()
		fmt.Printf("masbench v%s\n", version)
	},
}

func getVersion() string {
	data, err := os.ReadFile("VERSION")
	if err != nil {
		return "unknown"
	}
	return strings.TrimSpace(string(data))
}
