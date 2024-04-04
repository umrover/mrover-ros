#!/usr/bin/env python3
# \/\/\/\/ Help displayed below: \/\/\/\/
help = """
moteusConfigConvert: a script to help with saving and retrieving past moteus configs

How to use:

To save a config: 
    ./scripts/moteusConfigConvert save

To list configs saved:
    ./scripts/moteusConfigConvert list

To flash a saved config onto the moteus:
    ./scripts/moteusConfigConvert flash

Then, follow the provided instructions.

To retrieve a saved config:

"""
import sys
import shutil
import os
import subprocess
from pathlib import Path

MOTEUS_SAVE_DIR =  str(Path.cwd()) + "/config/moteus/" # Need to change this for other laptops.


def main():

    command: str
    if len(sys.argv) >= 2:
        command = sys.argv[1]
    else:
        command = input("Please enter a command {save, flash, list}: ")

    if command == "save":
        saveFName = input("Please enter name for saved config: ")
        id = int(input("Please enter ID of moteus: "))

        result = subprocess.run(["moteus_tool", "-t", str(id), "--dump-config"], capture_output=True)
        if result.returncode != 0:
            print(
                "Moteus_tool returned an error trying to load config from moteus. Check moteus is correctly connected and that you chose the correct ID"
            )
            print("Moteus_tool stdout: ", result.stdout.decode())
            print("Moteus_tool stderr: ", result.stderr.decode())
            return

        saveFile = open(MOTEUS_SAVE_DIR + saveFName, "w")
        saveFile.write(result.stdout)
        saveFile.close()

        print(f"Saved config {saveFName}")

    elif command == "list":
        files = os.listdir(MOTEUS_SAVE_DIR)
        print("Saved configs:\n\t")
        print(*files, sep="\n\t")

    elif command == "flash":
        storedConfigName = input("Please enter name for config to flash: ")
        id = int(input("Please enter ID of moteus: "))

        files = os.listdir(MOTEUS_SAVE_DIR)
        if storedConfigName not in files:
            print("Error: given config name not in saved configs")
            return

        result = subprocess.run(
            ["moteus_tool", "-t", str(id), "--restore-config", MOTEUS_SAVE_DIR + storedConfigName], capture_output=True
        )
        if result.returncode != 0:
            print(
                "Moteus_tool returned an error trying to save config to moteus. Check moteus is correctly connected and that you chose the correct ID"
            )
            return

        print("Wrote config to moteus")

    else:
        print("Unknown command entered -  \/please see usage\/")
        print(help)


if __name__ == "__main__":
    main()
