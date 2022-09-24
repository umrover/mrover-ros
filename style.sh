#!/usr/bin/env bash

# Print each command, fail on unset variables
set -xu

RED='\033[0;31m'
NC='\033[0m'

## Check that all tools are installed

clang_format_executable=clang-format-12
clang_format_executable_path=$(which "$clang_format_executable")
if [ ! -x "$clang_format_executable_path" ]; then
  echo -e "${RED}[Error] Please install clang-format with: sudo apt install ${clang_format_executable}${NC}"
  exit 1
fi


black_executable=black
black_executable_path=$(which "$black_executable")
if [ ! -x "$black_executable_path" ]; then
  echo -e "${RED}[Error] Please run pip3 install -r requirements.txt${NC}"
  exit 1
fi

# Style check Python with black
if ! black --version | grep -q 'black, 22.8.0'; then
  echo -e "${RED}[Error] Wrong black version${NC}"
  exit 1
fi


mypy_executable=mypy
mypy_executable_path=$(which "$mypy_executable")
if [ ! -x "$mypy_executable_path" ]; then
  echo -e "${RED}[Error] Please run pip3 install -r requirements.txt${NC}"
  exit 1
fi

if ! mypy --version | grep -q 'mypy 0.971'; then
  echo -e "${RED}[Error] Wrong mypy version${NC}"
  exit 1
fi

## Run checks

# Fail immediately if any command below fails
set -Eeo pipefail

echo "Style checking C++ ..."
find ./src -regex '.*\.\(cpp\|hpp\|h\)' -exec "$clang_format_executable_path" --dry-run -style=file -i {} \;
echo "Done"

echo "Style checking Python with black ..."
"$black_executable_path" --check --diff --line-length=120 ./src ./scripts
echo "Done"

echo "Style checking Python with mypy ..."
"$mypy_executable_path" --config-file mypy.ini --check ./src ./scripts
echo "Done"
