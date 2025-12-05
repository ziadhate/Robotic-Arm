import cv2
import numpy as np
import serial
import time

# ===== Serial Configuration =====
# Configure the serial port and baudrate used to communicate with the Arduino.
# Change PORT to match the COM port your Arduino is connected to (e.g., 'COM3', 'COM4', '/dev/ttyUSB0').
# BAUD must match the serial baud rate used in the Arduino sketch (here it's 115200).
PORT = 'COM4'   # عدّل على حسب البورت
BAUD = 115200

ser = None
try:
    # Try to open the serial port. `timeout=1` ensures read operations won't block forever.
    ser = serial.Serial(PORT, BAUD, timeout=1)
    # Wait a short time for the serial connection to initialize.
    time.sleep(2)
    # Clear any buffered data to avoid reading stale bytes.
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"Connected on {PORT}")
except Exception as e:
    # If opening the port fails (e.g., device not connected), switch to simulation mode.
    # In simulation mode the program continues but will not send serial commands.
    print(f"Connection failed on {PORT}. Error: {e}")
    ser = None
    print("Running in simulation mode")

# ===== Surface Calibration =====
# The user must click four points to define the sorting surface in the camera view.
# `points` will hold the four corner coordinates selected by mouse clicks.
points = []

def click(event, x, y, flags, param):
    # Mouse callback for selecting surface corner points.
    # When the left mouse button is clicked, record the point until there are 4 points.
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            print(f"Point {len(points)} recorded: ({x},{y})")

# Initialize camera capture (default camera index 0).
cap = cv2.VideoCapture(0)
# Optionally request a resolution; the downstream mask and coord mapping assumes 640x480.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create a window for selecting the surface and set the mouse callback to `click`.
cv2.namedWindow("Select Surface")
cv2.setMouseCallback("Select Surface", click)

# Loop until the user clicks 4 points or presses 'q'.
while len(points) < 4:
    ret, frame = cap.read()
    if not ret: break
    temp = frame.copy()
    # Draw any already-recorded points as green filled circles to guide the user.
    for p in points:
        cv2.circle(temp, p, 10, (0,255,0), -1)
    # Instruction text: show which point the user should click next.
    cv2.putText(temp, f"Click Point {len(points)+1}/4 on surface", (20,50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255),3)
    cv2.imshow("Select Surface", temp)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
# If user didn't provide 4 points, gracefully exit and release resources.
if len(points)<4:
    print("Not enough points selected. Exiting.")
    cap.release()
    if ser: ser.close()
    exit()

# ===== Surface mask =====
# Build a binary mask for the selected surface polygon. This mask will be used to ignore
# objects outside the surface area (speeds up processing and avoids false detections).
surface_mask = np.zeros((480,640), np.uint8)  # same size as camera frames
pts_array = np.array(points,np.int32)
cv2.fillPoly(surface_mask,[pts_array],255)
print("Surface mask created successfully.")

# ===== Zero Position Calibration =====
# User clicks a single point on the surface that acts as the origin (0,0) for mapping pixel coords
# to real-world coordinates (centimeters). This lets you define relative X,Y positions on the surface.
zero_point = None
print("Click on the Zero Position (origin) on the surface.")

def click_zero(event, x, y, flags, param):
    global zero_point
    if event == cv2.EVENT_LBUTTONDOWN:
        zero_point = (x, y)
        print(f"Zero Position recorded at: {zero_point}")

cv2.namedWindow("Zero Position")
cv2.setMouseCallback("Zero Position", click_zero)

# Wait until the zero point is set by clicking on the displayed camera frame.
while zero_point is None:
    ret, frame = cap.read()
    if not ret: break
    temp = frame.copy()
    # Draw the surface polygon to show where the zero position must be inside.
    cv2.polylines(temp,[np.array(points,np.int32)],True,(0,255,0),5)
    cv2.putText(temp,"Click ZERO position on the surface",(20,50),
                cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,255),2)
    cv2.imshow("Zero Position", temp)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyWindow("Zero Position")
print(f"Zero position set at: {zero_point}")

# ===== Main Loop =====
# Continuously read frames, detect target objects, map positions to real-world coordinates,
# and send X,Y,Z packets to the Arduino for the sorter/robot.
while True:
    ret, frame = cap.read()
    if not ret: break

    # --- Preprocessing ---
    # Apply Gaussian blur to reduce camera noise and small variations that may cause false contours.
    blurred = cv2.GaussianBlur(frame,(9,9),0)
    # Convert to HSV color space because it's more robust for color segmentation than BGR.
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # --- Red Mask ---
    # Create two ranges for red color: lower-red and upper-red in HSV.
    # Red wraps around the hue channel (0 and 180), so we need both low and high ranges.
    mask_red_low = cv2.inRange(hsv,(0,70,50),(10,255,255))
    mask_red_high = cv2.inRange(hsv,(160,70,50),(180,255,255))
    mask_red = mask_red_low + mask_red_high

    # --- Other objects ---
    # A broad mask for everything in the scene (used later to get non-red objects).
    # This range basically captures most visible pixels except very dark ones; tune if needed.
    mask_other = cv2.inRange(hsv,(0,40,30),(180,255,255))

    # --- Morphology ---
    # Use morphological closing to fill small holes and join nearby regions in the binary masks.
    # Kernel size (11,11) is relatively large; it smooths detections at the cost of small detail.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE,kernel,iterations=2)
    mask_other = cv2.morphologyEx(mask_other, cv2.MORPH_CLOSE,kernel,iterations=2)
    # `mask_non_red` is the set difference: everything except red (within the camera view range).
    mask_non_red = cv2.bitwise_and(mask_other, cv2.bitwise_not(mask_red))

    # --- Apply surface mask ---
    # Now ignore anything outside the user-defined surface polygon by masking both red and non-red masks.
    mask_red = cv2.bitwise_and(mask_red, surface_mask)
    mask_non_red = cv2.bitwise_and(mask_non_red, surface_mask)

    # --- Find contours ---
    # Extract contours on the binary masks. We use RETR_EXTERNAL to find only outer contours.
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    non_red_contours, _ = cv2.findContours(mask_non_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = cy = 0
    target_found = False
    action = ""

    # --- Determine target ---
    # Decide which object to sort based on contour area thresholds.
    # You currently choose RED objects first (priority). If a sufficiently large red contour exists,
    # we consider that the target; otherwise we check non-red objects.
    # The area threshold of 800 filters tiny blobs/noise; adjust to your object size and camera distance.
    if any(cv2.contourArea(c) > 800 for c in red_contours):
        # Select the largest red contour (most likely the intended object).
        c = max(red_contours,key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        cx,cy = x+w//2, y+h//2
        # Draw a red rectangle around the detected object for visualization.
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),6)
        cv2.putText(frame,"RED -> RIGHT",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),4)
        action = "RIGHT"
        target_found = True
    elif any(cv2.contourArea(c) > 800 for c in non_red_contours):
        # If no red object, check non-red objects and treat them as the other class.
        c = max(non_red_contours,key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        cx,cy = x+w//2, y+h//2
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),6)
        cv2.putText(frame,"OTHER -> LEFT",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),4)
        action = "LEFT"
        target_found = True

    # --- Map pixel to real-world coordinates (cm) relative to Zero Position ---
    # If a target is detected, convert the pixel center (cx,cy) into a physical coordinate system.
    # The code maps the bounding box center to a 0..20 cm range along both X and Y.
    # This mapping is linear interpolation from the pixel range (surface extents) to the real range (0-20 cm).
    if target_found and zero_point is not None:
        # Compute the min/max pixel extents of the selected polygon (surface).
        left = min(p[0] for p in points)
        right = max(p[0] for p in points)
        top = min(p[1] for p in points)
        bottom = max(p[1] for p in points)

        # Use numpy.interp to map (cx - zero_x) from the surface pixel extents to 0..20 cm.
        # Note: subtracting zero_point shifts coordinates so zero_point becomes the origin (0,0).
        X = np.interp(cx - zero_point[0],[left-zero_point[0], right-zero_point[0]],[0,20])
        Y = np.interp(cy - zero_point[1],[top-zero_point[1], bottom-zero_point[1]],[0,20])
        Z = 0  # Height is fixed here; change if you implement depth sensing.

        # --- Send to Arduino ---
        # Format a simple text packet 'X{:.2f} Y{:.2f} Z{:.2f}\n' that the Arduino can parse easily.
        # The newline at the end helps the Arduino know when a full packet has arrived.
        packet = f"X{X:.2f} Y{Y:.2f} Z{Z:.2f}\n"
        if ser:
            # Write bytes to serial only if serial connection is available.
            ser.write(packet.encode())
        # Print to console for debugging / logging even in simulation mode.
        print("Sending:",packet.strip())

    # --- Visual Feedback ---
    # Draw the surface polygon and the zero point on the camera preview so the operator knows
    # where the origin is and the boundaries of the working surface.
    cv2.polylines(frame,[np.array(points,np.int32)],True,(0,255,0),5)
    # Show the zero point as a filled blue circle.
    cv2.circle(frame, zero_point, 8, (255,0,0), -1)  # Show Zero Position
    cv2.putText(frame,"Sorting: RED->RIGHT | OTHER->LEFT",(20,70),
                cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)
    # Show the result window with annotated frame.
    cv2.imshow("Sorting + X,Y,Z Output", frame)

    # Press 'q' to quit the main loop and clean up.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up resources: release camera, close windows, and close serial port if open.
cap.release()
cv2.destroyAllWindows()
if ser: ser.close()
