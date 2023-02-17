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
        output_file = open(output_filename, "w+")

        print("Enter target moteus ID:")
        target_id = int(input())
        print("Default configuration is aux2 pin0 as hall pin.")
        print("Would you like to configure aux2 pin1 as a hall pin instead? (y/n)")
        aux2_config = input()

        pin0mode = 6
        pin0pull = 1
        pin1mode = 0
        pin1pull = 0
        if aux2_config == "y":
            pin0mode = 0
            pin0pull = 0
            pin1mode = 6
            pin1pull = 1

        if target_id >= 0 or target_id <= 8:
            while 1:
                line = input_file.readline()
                if line == "\n" or line == "":
                    break

                # check for id field
                if line[0:5] == "id.id":
                    line = "id.id " + str(target_id) + "\n"

                # check for aux2 pin0 mode
                elif line[0:16] == "aux2.pins.0.mode":
                    line = "aux2.pins.0.mode " + str(pin0mode) + "\n"

                # check for aux2 pin0 pull
                elif line[0:16] == "aux2.pins.0.pull":
                    line = "aux2.pins.0.pull " + str(pin0pull) + "\n"

                # check for aux2 pin1 mode
                elif line[0:16] == "aux2.pins.1.mode":
                    line = "aux2.pins.1.mode " + str(pin1mode) + "\n"

                # check for aux2 pin1 pull
                elif line[0:16] == "aux2.pins.1.pull":
                    line = "aux2.pins.1.pull " + str(pin1pull) + "\n"

                output_file.write("conf set " + line)

            input_file.close()
            output_file.close()
            print(
                f"\nDone converting config \n Run:\n sudo moteus_tool -t [X] --write-config {output_filename} \n to load onto moteus."
            )
        else:
            print(f"Target ID out of range: 0 <= {target_id} <= 8 \n Write cancelled.")


if __name__ == "__main__":
    main()
