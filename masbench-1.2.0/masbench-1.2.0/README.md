# masbench

**masbench** is a command-line tool for benchmarking multi-agents client in AI and Multi Agent System course in DTU.
It is designed to help students evaluate the performance of their client implementations.

Course information: [Artificial Intelligence and Multi-Agent Systems](https://lifelonglearning.dtu.dk/en/compute/single-course/artificial-intelligence-and-multi-agent-systems/)

Documentation is available at [https://marlonbando.github.io/masbench/](https://marlonbando.github.io/masbench/)

## Development Setup

### Prerequisites

- **Go 1.23.0 or later** (this project uses Go 1.23.0 with toolchain 1.23.11)
- **Git** for version control

### Installing Go

If you don't have Go installed, follow these steps:

#### Linux/macOS
```bash
# Download and install Go from the official website
# Visit: https://golang.org/dl/

# Or use a package manager:
# Ubuntu/Debian:
sudo apt update && sudo apt install golang-go

# macOS with Homebrew:
brew install go

# Arch Linux:
sudo pacman -S go
```

#### Windows
Download and install Go from the [official website](https://golang.org/dl/).

### Setting up the Development Environment

1. **Verify Go installation:**
   ```bash
   go version
   # Should output: go version go1.23.x linux/amd64 (or your platform)
   ```

2. **Download dependencies:**
   ```bash
   go mod download
   ```

3. **Build the project:**
   ```bash
   go build -o masbench .
   ```

4. **Run the application:**
   ```bash
   ./masbench --help
   ```