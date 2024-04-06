#!/usr/bin/env python3

"""
Script for converting moteus config into a writable format.
Takes in file generated from:

sudo moteus_tool -t X --dump-config

and adds "conf set " to every line. Generates "filename_write.cfg"
Write the config to a moteus with:

sudo moteus_tool -t X --write-config filename_write.cfg

Program input prompts for moteus ID, and aux pin configurations.

"""


def main():
    print("To save a particular configuration, use sudo moteus_tool -t X --dump-config > filename.cfg")
    print("Enter source config filename: ")
    input_filename = input()
    output_filename = input_filename.replace(".cfg", "") + "_write.cfg"

    with open(input_filename, "r") as input_file:
        with open(output_filename, "w+") as output_file:
            print("Enter target moteus ID:")
            target_id = int(input())
            if target_id < 0 or target_id > 8:
                print(f"Target ID out of range: 0 <= {target_id} <= 8 \nWrite cancelled.")
                return

            print("Default configuration is aux2 pin0 as hall pin.")
            print("Would you like to configure aux2 pin1 as a hall pin instead? (y/N)")
            aux2_config = input().lower()

            if aux2_config == "y" or aux2_config == "yes":
                pin0mode = 0
                pin0pull = 0
                pin1mode = 6
                pin1pull = 1
            else:
                pin0mode = 6
                pin0pull = 1
                pin1mode = 0
                pin1pull = 0

            params = {
                "id.id ": target_id,
                "aux2.pins.0.mode ": pin0mode,
                "aux2.pins.0.pull ": pin0pull,
                "aux2.pins.1.mode ": pin1mode,
                "aux2.pins.1.pull ": pin1pull,
            }

            while 1:
                line = input_file.readline()
                if line == "\n" or line == "":
                    break

                for key in params:
                    if line.startswith(key):
                        line = key + str(params[key]) + "\n"

                output_file.write("conf set " + line)

            print(
                f"\nDone converting config \n Run:\n sudo moteus_tool -t {target_id} --write-config {output_filename} \n to load onto moteus."
            )


if __name__ == "__main__":
    main()
