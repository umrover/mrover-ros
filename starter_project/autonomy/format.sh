#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED='\033[0;31m'
readonly NC='\033[0m'

BLACK_ARGS=(
  "--line-length=120"
  "--color"
)

# Just do a dry run if the "fix" argument is not passed
if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  BLACK_ARGS+=("--diff") # Show difference
  BLACK_ARGS+=("--check") # Exit with non-zero code if changes are required (for CI)
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

readonly BLACK_PATH=$(find_executable black 23.9.1)

echo
echo "Style checking Python with black ..."
"${BLACK_PATH}" "${BLACK_ARGS[@]}" ./src
