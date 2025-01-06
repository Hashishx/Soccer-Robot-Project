import cv2
import numpy as np
import socket
import json
import threading
import time
from queue import Queue

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
robot_ip = '192.168.1.103'  # Replace with the actual IP address of your robot
robot_port = 4210

# Shared variables
command_lock = threading.Lock()
last_command = None  # Store the last command to avoid redundant sends
last_command_time = 0  # Timestamp of the last command
command_interval = 0.5  # Minimum 500ms between commands

# Flag to control the infrared sensor state
infrared_enabled = True  # Set to True to enable infrared detection

# Create a queue for video frames
frame_queue = Queue(maxsize=2)  # Buffer for frames

# Frame skipping variable
frame_skip = 1  # Skip every n frames


def send_command(action, direction=None, ir_enabled=None):
    """
    Send a command to the ESP device with rate limiting and avoiding redundant commands.
    """
    global last_command, last_command_time
    current_time = time.time()

    # Handle infrared command separately
    if ir_enabled is not None and action == "ir_status":  # Infrared-specific command
        command = {"action": action, "infrared_enabled": ir_enabled}
    elif direction is not None and action == "move":  # Movement commands
        command = {"action": action, "direction": direction}
    else:
        command = {"action": action}  # Generic command (without direction or infrared status)

    if current_time - last_command_time > command_interval:
        with command_lock:
            if command != last_command:
                try:
                    message = json.dumps(command)
                    sock.sendto(message.encode(), (robot_ip, robot_port))
                    print(f"Sent command: {message}")
                    last_command = command
                    last_command_time = current_time
                except socket.error as e:
                    print(f"Socket error: {e}")


def capture_frames():
    """
    Capture video frames asynchronously from an IP camera and push them into the queue.
    """
    url = "http://192.168.1.107:8080/video"  # Replace with your IP camera URL
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            if frame_queue.full():
                frame_queue.get()  # Remove oldest frame if the queue is full

            frame_queue.put(frame)  # Add new frame to the queue
    finally:
        cap.release()


def video_processing():
    """
    Process frames from the capture queue to detect red objects and send movement commands.
    """
    global frame_skip
    frame_count = 0

    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()  # Get the next frame from the queue

            # Skip frames if necessary (to reduce processing load)
            if frame_count % frame_skip == 0:
                # Convert the frame to HSV color space
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Define the refined color range for red
                lower_red1 = (0, 160, 160)
                upper_red1 = (10, 255, 255)
                lower_red2 = (170, 160, 160)
                upper_red2 = (179, 255, 255)

                # Create masks for the specified color ranges
                mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

                # Combine the masks
                mask = cv2.bitwise_or(mask1, mask2)

                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    # Find the largest contour
                    largest_contour = max(contours, key=cv2.contourArea)

                    # Get the bounding box of the largest contour
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    # Calculate the center of the bounding box
                    center_x = x + w // 2

                    # Send command to ESP based on the position of the red object
                    frame_center_x = frame.shape[1] // 2
                    margin = int(frame.shape[1] * 0.1)  # 10% margin

                    if center_x < frame_center_x - margin:
                        send_command("move", direction="left")
                    elif center_x > frame_center_x + margin:
                        send_command("move", direction="right")
                    else:
                        send_command("move", direction="forward")
                else:
                    # No red object detected, rotate left to search
                    send_command("move", direction="left")

            frame_count += 1
        
            # Optional: Show the processed video feed (if debugging)
            cv2.imshow('Processed Video', mask)
            cv2.imshow('Original Frame', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


def main():
    """
    Main function to start the video processing thread and handle program termination.
    """
    global infrared_enabled
    try:
        # Initially, enable infrared detection
        send_command("ir_status", ir_enabled=infrared_enabled)

        # Start the video capturing thread
        capture_thread = threading.Thread(target=capture_frames)
        capture_thread.daemon = True  # Daemonize the thread to exit with the program
        capture_thread.start()

        # Start the video processing thread
        video_thread = threading.Thread(target=video_processing)
        video_thread.daemon = True  # Daemonize the thread to exit with the program
        video_thread.start()

        # Run the main loop
        while True:
            time.sleep(1)  # Keep the main thread alive and manage the child threads

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        sock.close()
        print("Program terminated.")


if __name__ == "__main__":
    main()
