package parsers

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"masbench/internals/models"
	"os"
	"path/filepath"
	"regexp"
	"strings"
)

// ParseLogToCSV parses a log file and writes the extracted metrics to a CSV file.
func ParseLogToCSV(logFilePath string, outputFilePath string) error {
	file, err := os.Open(logFilePath)
	if err != nil {
		return fmt.Errorf("error opening log file: %w", err)
	}
	defer file.Close()

	levelPattern := regexp.MustCompile(`\[server\]\[info\] Running client on level file: ([^\s]+)`)
	solvedPattern := regexp.MustCompile(`\[server\]\[info\] Level solved: (Yes|No)`)
	actionsPattern := regexp.MustCompile(`\[server\]\[info\] Actions used: (\d{1,3}(?:,\d{3})*)`)
	timePattern := regexp.MustCompile(`\[server\]\[info\] Time to solve: ([0-9.]+) seconds`)
	exploredPattern := regexp.MustCompile(`\[client\]\[message\]\s*Explored:\s*(\d+)`)
	generatedPattern := regexp.MustCompile(`\[client\]\[message\]\s*Generated:\s*(\d+)`)
	memoryPattern := regexp.MustCompile(`\[client\]\[message\]\s*Alloc:\s*([0-9.]+)\s*MB`)
	maxMemoryPattern := regexp.MustCompile(`\[client\]\[message\]\s*MaxAlloc:\s*([0-9.]+)\s*MB`)

	var logs []models.LevelMetrics
	var currentLevel *models.LevelMetrics

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()

		if levelMatch := levelPattern.FindStringSubmatch(line); levelMatch != nil {
			if currentLevel != nil {
				logs = append(logs, *currentLevel)
			}
			levelName := strings.TrimSuffix(filepath.Base(levelMatch[1]), filepath.Ext(levelMatch[1]))
			currentLevel = &models.LevelMetrics{LevelName: levelName}
		}

		if currentLevel != nil {
			if solvedMatch := solvedPattern.FindStringSubmatch(line); solvedMatch != nil {
				currentLevel.Solved = solvedMatch[1]
			}
			if actionsMatch := actionsPattern.FindStringSubmatch(line); actionsMatch != nil {
				currentLevel.Actions = strings.ReplaceAll(actionsMatch[1], ",", "")
			}
			if timeMatch := timePattern.FindStringSubmatch(line); timeMatch != nil {
				currentLevel.Time = timeMatch[1]
			}
			if exploredMatch := exploredPattern.FindStringSubmatch(line); exploredMatch != nil {
				currentLevel.Explored = exploredMatch[1]
			}
			if generatedMatch := generatedPattern.FindStringSubmatch(line); generatedMatch != nil {
				currentLevel.Generated = generatedMatch[1]
			}
			if memoryMatch := memoryPattern.FindStringSubmatch(line); memoryMatch != nil {
				currentLevel.MemoryAlloc = memoryMatch[1]
			}
			if maxMemoryMatch := maxMemoryPattern.FindStringSubmatch(line); maxMemoryMatch != nil {
				currentLevel.MaxAlloc = maxMemoryMatch[1]
			}
		}
	}

	if currentLevel != nil {
		logs = append(logs, *currentLevel)
	}

	if err := scanner.Err(); err != nil {
		return fmt.Errorf("error reading log file: %w", err)
	}

	// Write to CSV
	csvFile, err := os.Create(outputFilePath)
	if err != nil {
		return fmt.Errorf("error creating CSV file: %w", err)
	}
	defer csvFile.Close()

	csvWriter := csv.NewWriter(csvFile)
	defer csvWriter.Flush()

	header := []string{"LevelName", "Solved", "Actions", "Time", "Generated", "Explored", "MemoryAlloc", "MaxAlloc"}
	if err := csvWriter.Write(header); err != nil {
		return fmt.Errorf("error writing CSV header: %w", err)
	}

	for _, log := range logs {
		row := []string{
			log.LevelName,
			log.Solved,
			log.Actions,
			log.Time,
			log.Generated,
			log.Explored,
			log.MemoryAlloc,
			log.MaxAlloc,
		}
		if err := csvWriter.Write(row); err != nil {
			return fmt.Errorf("error writing CSV row: %w", err)
		}
	}

	fmt.Printf("CSV file successfully created: %s\n", outputFilePath)
	return nil
}
