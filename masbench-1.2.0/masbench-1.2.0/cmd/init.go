package cmd

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"masbench/internals/models"

	"github.com/spf13/cobra"
	"gopkg.in/yaml.v3"
)

const (
	colorReset  = "[0m"
	colorYellow = "[33m"
	colorGreen  = "[32m"
	colorRed    = "[31m"
	colorWhite  = "[37m"
)

func init() {
	rootCmd.AddCommand(initCmd)
}

var initCmd = &cobra.Command{
	Use:   "init",
	Short: "Initialize masbench in this repository",
	Long: "This command sets up the necessary configuration files and directories for masbench " +
		"in the current repository.",
	Run: func(cmd *cobra.Command, args []string) {
		runInit()
	},
}

func runInit() {
	currentDir, err := os.Getwd()
	if err != nil {
		fmt.Printf("%sError getting current directory: %v%s\n", colorRed, err, colorReset)
		return
	}

	fmt.Printf("%smasbench initialization will create masbench_config.yml in %s%s\n", colorWhite, currentDir, colorReset)
	if !askForConfirmation("Do you want to continue? [Y/n]\n") {
		fmt.Printf("%sInitialization cancelled.%s\n", colorRed, colorReset)
		return
	}

	prompt := "Do you want to configure the file with an interactive dialog?\n" +
		"(If not, a default file will be created for manual edit) [Y/n]\n"
	if askForConfirmation(prompt) {
		interactiveConfigCreation(currentDir)
	} else {
		createDefaultConfig(currentDir)
	}
}

func createDefaultConfig(basePath string) {
	defaultConfig := models.Config{
		ServerPath:      "path/to/your/server_executable",
		LevelsDir:       "path/to/your/levels_directory",
		BenchmarkFolder: "benchmarks",
		ClientCommand:   "your_client_command --level {level_path}",
		Timeout:         180,
	}

	writeConfig(basePath, &defaultConfig)
	fmt.Printf("%sCreated default config file at %s. Please edit it manually.%s\n", colorGreen, filepath.Join(basePath, "masbench_config.yml"), colorReset)
}

func interactiveConfigCreation(basePath string) {
	reader := bufio.NewReader(os.Stdin)
	fmt.Printf("%sPlease provide the following configuration values.%s\n", colorWhite, colorReset)

	serverPath := askForServerPath(reader)
	levelsDir := askForLevelsDir(reader)
	benchmarkFolder := askForBenchmarkFolder(reader)
	clientCommand := askForClientCommand(reader)
	
	newConfig := models.Config{
		ServerPath:      serverPath,
		LevelsDir:       levelsDir,
		BenchmarkFolder: benchmarkFolder,
		ClientCommand:   clientCommand,
		Timeout:         180, // Default timeout to simplify initialization
	}

	writeConfig(basePath, &newConfig)
	configPath := filepath.Join(basePath, "masbench_config.yml")
	fmt.Printf("%sCreated config file at %s.%s\n", colorGreen, configPath, colorReset)
}

func writeConfig(basePath string, cfg *models.Config) {
	yamlData, err := yaml.Marshal(cfg)
	if err != nil {
		fmt.Printf("%sError marshalling config: %v%s\n", colorRed, err, colorReset)
		return
	}

	configPath := filepath.Join(basePath, "masbench_config.yml")
	err = os.WriteFile(configPath, yamlData, 0644)
	if err != nil {
		fmt.Printf("%sError writing config file: %v%s\n", colorRed, err, colorReset)
	}
}

func askForLevelsDir(reader *bufio.Reader) string {
	prompt := "Enter the path to your level directory. It's recommended to create a separate folder " +
		"where you place only the levels that you want to benchmark to reduce the benchmark time."
	for {
		fmt.Printf("%s%s%s\n", colorYellow, prompt, colorReset)
		fmt.Printf("%sLevels directory path:%s\n", colorYellow, colorReset)
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		if input == "" {
			fmt.Printf("%sPath cannot be empty.%s\n", colorRed, colorReset)
			continue
		}

		info, err := os.Stat(input)
		if err != nil {
			if os.IsNotExist(err) {
				confirmationPrompt := fmt.Sprintf("Directory '%s' does not exist. Create it? [Y/n]\n", input)
				if askForConfirmation(confirmationPrompt) {
					if err := os.MkdirAll(input, 0755); err != nil {
						fmt.Printf("%sError creating directory '%s': %v%s\n", colorRed, input, err, colorReset)
						continue
					}
					fmt.Printf("%sCreated directory '%s'.%s\n", colorGreen, input, colorReset)
					return input
				} else {
					fmt.Printf("%sPlease enter a different path.%s\n", colorWhite, colorReset)
					continue
				}
			} else {
				fmt.Printf("%sError checking directory path '%s': %v%s\n", colorRed, input, err, colorReset)
				continue
			}
		}

		if !info.IsDir() {
			fmt.Printf("%sInvalid input: The path '%s' is not a directory.%s\n", colorRed, input, colorReset)
			continue
		}

		return input
	}
}

func askForServerPath(reader *bufio.Reader) string {
	for {
		fmt.Printf("%sEnter the path to your server executable (.jar file):%s\n", colorYellow, colorReset)
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		if !strings.HasSuffix(strings.ToLower(input), ".jar") {
			fmt.Printf("%sInvalid input: The path must be for a .jar file.%s\n", colorRed, colorReset)
			continue
		}

		if _, err := os.Stat(input); os.IsNotExist(err) {
			fmt.Printf("%sInvalid input: The file '%s' does not exist.%s\n", colorRed, input, colorReset)
			continue
		} else if err != nil {
			// Handle other potential errors from os.Stat, though for this case, we can just report and loop.
			fmt.Printf("%sError checking file path '%s': %v%s\n", colorRed, input, err, colorReset)
			continue
		}

		return input
	}
}

func askForString(reader *bufio.Reader, prompt, defaultValue string) string {
	fmt.Printf("%s%s [%s]:%s\n", colorYellow, prompt, defaultValue, colorReset)
	input, _ := reader.ReadString('\n')
	input = strings.TrimSpace(input)
	if input == "" {
		return defaultValue
	}
	return input
}

func askForConfirmation(prompt string) bool {
	reader := bufio.NewReader(os.Stdin)
	for {
		fmt.Printf("%s%s%s", colorYellow, prompt, colorReset)
		input, _ := reader.ReadString('\n')
		input = strings.ToLower(strings.TrimSpace(input))
		if input == "y" || input == "yes" || input == "" {
			return true
		}
		if input == "n" || input == "no" {
			return false
		}
		fmt.Printf("%sInvalid input. Please enter y/yes or n/no.%s\n", colorRed, colorReset)
	}
}

func askForBenchmarkFolder(reader *bufio.Reader) string {
	for {
		fmt.Printf("%sPress Enter to create the benchmark folder in the current directory or provide a custom path:%s\n", colorYellow, colorReset)

		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)

		if input == "" {
			currentDir, err := os.Getwd()
			if err != nil {
				fmt.Printf("%sError getting current directory: %v%s\n", colorRed, err, colorReset)
				return "benchmarks"
			}
			return filepath.Join(currentDir, "benchmarks")
		}

		if _, err := os.Stat(input); os.IsNotExist(err) {
			confirmationPrompt := fmt.Sprintf("Directory '%s' does not exist. Create it? [Y/n]\n", input)
			if askForConfirmation(confirmationPrompt) {
				if err := os.MkdirAll(input, 0755); err != nil {
					fmt.Printf("%sError creating directory '%s': %v%s\n", colorRed, input, err, colorReset)
					continue
				}
				fmt.Printf("%sCreated directory '%s'.%s\n", colorGreen, input, colorReset)
				return input
			} else {
				fmt.Printf("%sPlease enter a different path.%s\n", colorWhite, colorReset)
				continue
			}
		} else if err != nil {
			fmt.Printf("%sError checking directory path '%s': %v%s\n", colorRed, input, err, colorReset)
			continue
		}

		return input
	}
}

func askForClientCommand(reader *bufio.Reader) string {
	fmt.Printf("%sEnter the command to start your client.\nExample: \"python -m project.src.searchclient -greedy --max-memory 1024\"\n"+
		"Note: This is the command that goes after -c when running the server.\n"+
		"Do not include 'java -jar server.jar' or level path, just your client command:%s\n", colorYellow, colorReset)
	input, _ := reader.ReadString('\n')
	return strings.TrimSpace(input)
}