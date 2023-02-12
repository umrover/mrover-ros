"""
Script for converting moteus config into a writable format.
Takes in file generated from:

sudo moteus_tool -t X --dump-config

and adds "conf set " to every line. Generates "filename_write.cfg"
Write the config to a moteus with:

sudo moteus_tool -t X --write-config filename_write.cfg

"""
def main():
    print("Enter filename: ")
    input_filename = input()
    output_filename = input_filename.replace(".cfg", "") + "_write.cfg"
    
    with open(input_filename, "r") as input_file:
        output_file = open(output_filename, "w+")
        
        line = input_file.readline()
        while(line != ""):
            output_file.write("conf set " + line)
            
            line = input_file.readline()
            
        input_file.close()
        output_file.close()
    print(f"\nDone converting config \n Run:\n sudo moteus_tool -t [X] --write-config {output_filename} \n to load onto moteus.")
        

if __name__ == "__main__":
    main()