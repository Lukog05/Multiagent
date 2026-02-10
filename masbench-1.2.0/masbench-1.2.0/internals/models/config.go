package models

// Config holds the application configuration settings.
type Config struct {
	ServerPath      string `yaml:"ServerPath"`
	LevelsDir       string `yaml:"LevelsDir"`
	BenchmarkFolder string `yaml:"BenchmarkFolder"`
	ClientCommand   string `yaml:"ClientCommand"`
	Timeout         int    `yaml:"Timeout"`
}
