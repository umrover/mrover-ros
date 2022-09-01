format_executable=clang-format-12
format_executable_path=$(which "$format_executable")
if [ ! -x "$format_executable_path" ]; then
  echo "Please install clang-format with: sudo apt install $format_executable"
  exit 1
fi

find . -regex '.*\.\(cpp\|hpp\|h\)' -exec "$format_executable" --dry-run -style=file -i {} \;
