import cv2

# Create a VideoCapture object with CAP_GSTREAMER
cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=BGR ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

# Get the camera resolution
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Create a VideoWriter object with CAP_GSTREAMER
out = cv2.VideoWriter("output.mp4", cv2.CAP_GSTREAMER, 0, 30, (width, height))

# Set the video encoding parameters
out.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264"))  # Set the video codec to H.264
out.set(cv2.CAP_PROP_BITRATE, 1000000)  # Set the video bitrate to 1,000,000 bits per second
out.set(cv2.CAP_PROP_QUALITY, 0.8)  # Set the video quality (0-1)

# Start capturing frames from the camera
while True:
    ret, frame = cap.read()

    if ret:
        # Write the frame to the output video
        out.write(frame)

        # Display the frame
        cv2.imshow("frame", frame)

    # Check if the user wants to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the resources
cap.release()
out.release()
cv2.destroyAllWindows()