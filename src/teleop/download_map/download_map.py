from pathlib import Path
import zipfile
import json
import boto3
import botocore
import subprocess


# If System has issues with number of file watcher when using GUI with downloaded map
# Run "sudo sysctl -w fs.inotify.max_user_watches=100000"
def main():
    # runs rospack find mrover to get directory
    result = subprocess.Popen(["rospack", "find", "mrover"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Kills the child process if the timeout expires
    try:
        m_byte, err = result.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        result.kill()
        m_byte, err = result.communicate()

    # Error check when Popen dies and shows what is causing the error
    if result.returncode != 0:
        print("Failed to capture output. Return code:%d %s" % (result.returncode, err.decode("utf-8")[:-1]))
        exit(0)

    # Creating Path and strings needed for downloading and unzipping
    mrover = Path(m_byte.decode("utf-8")[:-1])  # load directory into path
    mapzip = Path(mrover / "src" / "teleop" / "gui" / "src" / "static" / "map.zip")
    stringmappath = Path(mrover / "src" / "teleop" / "gui" / "src" / "static" / "map")
    stringzip = mapzip.as_posix()
    stringmap = stringmappath.as_posix()

    # Check if path exists
    path_exists = Path.exists(mrover / "src" / "teleop" / "download_map" / "keys.json")
    if not path_exists:
        out = {"accessKey": "", "secretKey": ""}
        f = open(mrover / "src" / "teleop" / "download_map" / "keys.json", "w")
        json.dump(out, f, indent=4)
        print("Access Keys not found, input keys into keys.json")
        exit(0)

    # loads keys
    f = open(mrover / "src" / "teleop" / "download_map" / "keys.json", "r")
    keys = json.load(f)

    # download and verify
    print("Downloading map... (This should take around a minute)")
    try:
        s3 = boto3.client("s3", aws_access_key_id=keys["accessKey"], aws_secret_access_key=keys["secretKey"])
        s3.download_file("rover-map", "map.zip", stringzip)
    except botocore.exceptions.ClientError as error:
        print(error.response["Error"]["Code"])  # Error code
        print(error.response["Error"]["Message"])  # Error message
        exit(0)
    print("Map Download Complete, Unzipping... (This should take around a minute)")

    # Unzip and delete zip file
    with zipfile.ZipFile(mapzip, "r") as zip_ref:
        zip_ref.extractall(stringmap)
    Path(mapzip).unlink()  # remove the map.zip

    print("Map Download Complete and Unzipped!")


if __name__ == "__main__":
    main()
