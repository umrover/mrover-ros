import subprocess   # Run new processes
import re           # regex

# Returns a list of video devices
def generate_dev_list():
    # Runs bash script line: `find /dev -iname 'video*' -printf "%f\n"`
    dev_list = subprocess.run(["find", "/dev", "-iname", "video*", "-printf", "%f\n"], capture_output=True, text=True)
    ret_dev_list = list()
    
    # Look through /dev for files with "video*"
    for dev in dev_list.stdout.splitlines():
        dev_num = re.sub(r'video', '', dev.strip())
        # Runs bash script line: 'v4l2-ctl --list-formats --device /dev/$dev'
        cmd_output = subprocess.run(["v4l2-ctl", "--list-formats", "--device", f"/dev/{dev.strip()}"], capture_output=True, text=True)
        # Checks if video* file has [0], [1], etc. to see if it is an actual video capture source
        if re.search(r'\[[0-9]\]', cmd_output.stdout):
            ret_dev_list.append(dev_num)
    return ret_dev_list
# print(ret_dev_list)