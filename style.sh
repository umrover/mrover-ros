#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED='\033[0;31m'
readonly NC='\033[0m'

print_update_error() {
  echo -e "${RED}[Error] Please update with ./ansible.sh build.yml${NC}"
  exit 1
}

## Check that all tools are installed

readonly clang_format_executable=clang-format-16
readonly clang_format_executable_path=$(which "$clang_format_executable")
if [ ! -x "$clang_format_executable_path" ]; then
  echo -e "${RED}[Error] Could not find clang-format-16${NC}"
  print_update_error
fi


readonly black_executable=black
readonly black_executable_path=$(which "$black_executable")
if [ ! -x "$black_executable_path" ]; then
  echo -e "${RED}[Error] Could not find black${NC}"
  print_update_error
fi

# Style check Python with black
if ! black --version | grep -q 'black, 23.9.1'; then
  echo -e "${RED}[Error] Wrong black version${NC}"
  print_update_error
fi


readonly mypy_executable=mypy
readonly mypy_executable_path=$(which "$mypy_executable")
if [ ! -x "$mypy_executable_path" ]; then
  echo -e "${RED}[Error] Could not find mypy${NC}"
  print_update_error
fi

if ! mypy --version | grep -q 'mypy 1.5.1'; then
  echo -e "${RED}[Error] Wrong mypy version${NC}"
  print_update_error
fi

## Run checks

echo "Style checking C++ ..."
readonly FOLDERS=(
  ./src/perception
  ./src/gazebo
  ./src/util
)
for FOLDER in "${FOLDERS[@]}"; do
  find "${FOLDER}" -regex '.*\.\(cpp\|hpp\|h\)' -exec "$clang_format_executable_path" --dry-run -style=file -i {} \;
done
echo "Done"

echo
echo "Style checking Python with black ..."
"$black_executable_path" --line-length=120 ./src ./scripts

echo
echo "Style checking Python with mypy ..."
"$mypy_executable_path" --config-file mypy.ini --check ./src ./scripts
