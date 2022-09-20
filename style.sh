#!/usr/bin/env bash

# Print each command, fail on unset variables
set -xu

# Style check C++ with clang-format
clang_format_executable=clang-format-12
clang_format_executable_path=$(which "$clang_format_executable")
if [ ! -x "$clang_format_executable_path" ]; then
  echo "[Error] Please install clang-format with: sudo apt install $clang_format_executable"
  exit 1
fi

echo "Style checking C++ ..."
find . -regex '.*\.\(cpp\|hpp\|h\)' -exec "$clang_format_executable_path" --dry-run -style=file -i {} \;
echo "Done"

# Style check Python with black
black_executable=black
black_executable_path=$(which "$black_executable")
if [ ! -x "$black_executable_path" ]; then
  echo "[Error] Please run pip3 install -r requirements.txt"
  exit 1
fi

if ! black --version | grep -q 'black, 22.8.0'; then
  echo "[Error] Wrong black version"
  exit 1
fi

# TODO: don't hardcode settings!

echo "Style checking Python with black ..."
"$black_executable_path" --check --diff --line-length=120 src scripts
echo "Done"

# Style check Python with mypy
mypy_executable=mypy
mypy_executable_path=$(which "$mypy_executable")
if [ ! -x "$mypy_executable_path" ]; then
  echo "[Error] Please run pip3 install -r requirements.txt"
  exit 1
fi

if ! mypy --version | grep -q 'mypy 0.971'; then
  echo "[Error] Wrong mypy version"
  exit 1
fi

echo "Style checking Python with mypy ..."
"$mypy_executable_path" --config-file mypy.ini --check src scripts
echo "Done"

# Style check Vue with eslint, local install in node_modules
echo "Style checking Vue.js with eslint"
eslint_executable_path=$(rospack find mrover)/src/teleop/gui/node_modules/.bin/
if [ -d "$eslint_executable_path" ]; then
  eslint_executable_path="$eslint_executable_path"eslint
else
  echo "[Error] Could not find eslint, please run src/teleop/gui/gui_install.sh script"
  exit 1
fi
"$eslint_executable_path" ./src/teleop/gui/src --ext .vue,.js
echo "Done"
