"""
Script for converting moteus config into a writable format.
Takes in file generated from:

sudo moteus_tool -t X --dump-config

and adds "conf set " to every line. Generates "filename_write.cfg"
Write the config to a moteus with:

sudo moteus_tool -t X --write-config filename_write.cfg

"""
def main():
    print("To save a particular configuration, use sudo moteus_tool -t X --dump-config > filename.cfg")
    print("Enter filename: ")
    input_filename = input()
    output_filename = input_filename.replace(".cfg", "") + "_write.cfg"
    
    with open(input_filename, "r") as input_file:
        output_file = open(output_filename, "w+")
        
        print("Enter target moteus ID:")
        target_id = int(input())
        print("Enter aux2 pin3 mode: (0 or 6)")
        target_pin3_mode = input()
        print("Enter aux2 pin3 pull: (0 or 1)")
        target_pin3_pull = input()
        
        if(target_id >= 0 or target_id <= 8):
        
            while(1):
                line = input_file.readline()
                if(line == "\n" or line == ""):
                    break
                
                # check for id field
                if(line[0:5] == "id.id"):
                    line = "id.id " + str(target_id) + "\n"
                    
                # check for aux2 pin3 mode
                if(line[0:16] == "aux2.pins.3.mode"):
                    line = "aux2.pins.3.mode " + str(target_pin3_mode) + "\n"
                    
                # check for aux2 pin3 pull
                if(line[0:16] == "aux2.pins.3.pull"):
                    line = "aux2.pins.3.pull " + str(target_pin3_pull) + "\n"
        
                output_file.write("conf set " + line)
                
            input_file.close()
            output_file.close()
            print(f"\nDone converting config \n Run:\n sudo moteus_tool -t [X] --write-config {output_filename} \n to load onto moteus.")
        else:
            print(f"Target ID out of range: 0 <= {target_id} <= 8 \n Write cancelled.")

if __name__ == "__main__":
    main()
