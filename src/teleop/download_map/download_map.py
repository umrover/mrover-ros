from pathlib import Path
import zipfile
import json
import boto3
import subprocess


# If System has issues with number of file watcher when using GUI with downloaded map
# Run "sudo sysctl -w fs.inotify.max_user_watches=100000"
def main():
    result = subprocess.Popen(
        ["rospack", "find", "mrover"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )  # runs rospack find mrover to get directory
    m_bytes, err = result.communicate()
    m = m_bytes.decode("utf-8")  # convert byte to string type
    m = m[: len(m) - 1]  # the old m includes the '\n' and this gets rid of that
    mrover = Path(m)  # load directory into path
    path_exists = Path.exists(mrover / "src" / "teleop" / "download_map" / "keys.json")
    if not path_exists:
        out = {"accessKey": "", "secretKey": ""}
        f = open(mrover / "src" / "teleop" / "download_map" / "keys.json", "w")
        json.dump(out, f, indent=4)
        print("Access Keys not found, input keys into keys.json")
        exit(0)
    f = open(mrover / "src" / "teleop" / "download_map" / "keys.json", "r")
    keys = json.load(f)
    s3 = boto3.client("s3", aws_access_key_id=keys["accessKey"], aws_secret_access_key=keys["secretKey"])
    print("Downloading map... (This should take around a minute)")
    mapzip = Path(mrover / "src" / "teleop" / "gui" / "src" / "static" / "map.zip")
    stringzip = mapzip.as_posix()  # string of map.zip path
    stringmappath = mrover / "src" / "teleop" / "gui" / "src" / "static" / "map"
    stringmap = stringmappath.as_posix()  # string of map path
    # print(stringzip)
    s3.download_file("rover-map", "map.zip", stringzip)
    print("Map Download Complete, Unzipping... (This should take around a minute)")
    with zipfile.ZipFile(mapzip, "r") as zip_ref:
        zip_ref.extractall(stringmap)
    Path(mapzip).unlink()  # remove the map.zip
    print("Map Download Complete!")


if __name__ == "__main__":
    main()
