find . -regex '.*\.\(cpp\|hpp\|h\)' -exec clang-format-12 -style=file -i {} \;
