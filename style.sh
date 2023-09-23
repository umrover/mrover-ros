#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED='\033[0;31m'
readonly NC='\033[0m'

BLACK_ARGS=(
  "--line-length=120"
  "--color"
)
CLANG_FORMAT_ARGS=(
  "-style=file"
)

# Just do a dry run if the "fix" argument is not passed
if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  BLACK_ARGS+=("--diff") # Show difference
  BLACK_ARGS+=("--check") # Exit with non-zero code if changes are required (for CI)
  CLANG_FORMAT_ARGS+=("--dry-run")
fi

function print_update_error() {
  echo -e "${RED}[Error] Please update with ./ansible.sh build.yml${NC}"
  exit 1
}

function find_executable() {
  local readonly executable="$1"
  local readonly version="$2"
  local readonly path=$(which "${executable}")
  if [ ! -x "${path}" ]; then
    echo -e "${RED}[Error] Could not find ${executable}${NC}"
    print_update_error
  fi
  if ! "${path}" --version | grep -q "${version}"; then
    echo -e "${RED}[Error] Wrong ${executable} version${NC}"
    print_update_error
  fi
  echo "${path}"
}

## Check that all tools are installed

readonly CLANG_FORMAT_PATH=$(find_executable clang-format-16 16.0)
readonly BLACK_PATH=$(find_executable black 23.9.1)
readonly MYPY_PATH=$(find_executable mypy 1.5.1)

## Run checks

echo "Style checking C++ ..."
readonly FOLDERS=(
  ./src/perception
  ./src/gazebo
  ./src/util
)
for FOLDER in "${FOLDERS[@]}"; do
  find "${FOLDER}" -regex '.*\.\(cpp\|hpp\|h\)' -exec "${CLANG_FORMAT_PATH}" "${CLANG_FORMAT_ARGS[@]}" -i {} \;
done
echo "Done"

echo
echo "Style checking Python with black ..."
"$BLACK_PATH" "${BLACK_ARGS[@]}" ./src ./scripts

echo
echo "Style checking Python with mypy ..."
"$MYPY_PATH" --config-file=mypy.ini --check ./src ./scripts
