package utils

import (
	"fmt"
	"os"
	"strconv"

	"github.com/go-gota/gota/dataframe"
)

// LoadCSV reads a CSV file and returns a DataFrame
func LoadCSV(filePath string) (dataframe.DataFrame, error) {
	file, err := os.Open(filePath)
	if err != nil {
		return dataframe.DataFrame{}, fmt.Errorf("failed to open CSV file: %w", err)
	}
	defer file.Close()

	df := dataframe.ReadCSV(file)
	return df, nil
}

// ToMap converts a DataFrame to a nested map structure keyed by level name
// Returns: map[levelName]map[columnName]value
func ToMap(df dataframe.DataFrame) map[string]map[string]string {
	result := make(map[string]map[string]string)
	for i := 0; i < df.Nrow(); i++ {
		levelName := df.Elem(i, 0).String()
		rowData := make(map[string]string)
		for j, colName := range df.Names() {
			rowData[colName] = df.Elem(i, j).String()
		}
		result[levelName] = rowData
	}
	return result
}

// GetColumnIndex finds the column index by name, returns -1 if not found
func GetColumnIndex(df dataframe.DataFrame, colName string) int {
	for i, name := range df.Names() {
		if name == colName {
			return i
		}
	}
	return -1
}

// GetFloatFromDF extracts a float64 value from a DataFrame at specified row and column
// Returns 0.0 if column not found or parse error
func GetFloatFromDF(df dataframe.DataFrame, row int, colName string) float64 {
	colIndex := GetColumnIndex(df, colName)
	if colIndex == -1 {
		return 0.0
	}

	valStr := df.Elem(row, colIndex).String()
	val, err := strconv.ParseFloat(valStr, 64)
	if err != nil {
		return 0.0
	}

	return val
}

// GetFloatFromMap extracts a float64 value from a map
// Returns 0.0 if key not found or parse error
func GetFloatFromMap(data map[string]string, colName string) float64 {
	valStr, ok := data[colName]
	if !ok {
		return 0.0
	}

	val, err := strconv.ParseFloat(valStr, 64)
	if err != nil {
		return 0.0
	}

	return val
}

// GetStringFromDF extracts a string value from a DataFrame at specified row and column
// Returns empty string if column not found
func GetStringFromDF(df dataframe.DataFrame, row int, colName string) string {
	colIndex := GetColumnIndex(df, colName)
	if colIndex == -1 {
		return ""
	}

	return df.Elem(row, colIndex).String()
}

// GetStringFromMap extracts a string value from a map
// Returns empty string if key not found
func GetStringFromMap(data map[string]string, colName string) string {
	val, ok := data[colName]
	if !ok {
		return ""
	}

	return val
}
