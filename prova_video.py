import cv2
import numpy as np
import math

def rotate_image(image):
    """Rotate the image 90° counterclockwise if needed (optional)."""
    return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

def compute_gradients(image):
    """
    Compute horizontal (Ix) and vertical (Iy) gradients using central differences.
    """
    Ix = (np.roll(image, -1, axis=1) - np.roll(image, 1, axis=1)) / 2.0
    Iy = (np.roll(image, -1, axis=0) - np.roll(image, 1, axis=0)) / 2.0
    return Ix, Iy

def compute_optical_flow_lk(prev_frame, next_frame, window_size=5, step=16):
    """
    Compute optical flow using Lucas-Kanade approach on a grid of points.
    Converts frames to grayscale and computes the flow vectors between them.
    """
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    next_gray = cv2.cvtColor(next_frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    Ix, Iy = compute_gradients(prev_gray)
    It = next_gray - prev_gray

    h, w = prev_gray.shape
    flow_vectors = []
    margin = window_size // 2

    for y in range(margin, h - margin, step):
        for x in range(margin, w - margin, step):
            window_Ix = Ix[y-margin:y+margin+1, x-margin:x+margin+1].flatten()
            window_Iy = Iy[y-margin:y+margin+1, x-margin:x+margin+1].flatten()
            window_It = It[y-margin:y+margin+1, x-margin:x+margin+1].flatten()

            sum_Ix2 = np.sum(window_Ix ** 2)
            sum_Iy2 = np.sum(window_Iy ** 2)
            sum_IxIy = np.sum(window_Ix * window_Iy)
            sum_IxIt = np.sum(window_Ix * window_It)
            sum_IyIt = np.sum(window_Iy * window_It)

            det = sum_Ix2 * sum_Iy2 - sum_IxIy ** 2
            if det != 0:
                u = (-sum_Iy2 * sum_IxIt + sum_IxIy * sum_IyIt) / det
                v = (-sum_IxIy * sum_IxIt + sum_Ix2 * sum_IyIt) / det
            else:
                u, v = 0, 0
            flow_vectors.append((x, y, u, v))
    return flow_vectors

def draw_optical_flow(flow_vectors, frame):
    """
    Draw the optical flow vectors on the frame.
    """
    for (x, y, u, v) in flow_vectors:
        cv2.line(frame, (x, y), (int(x + u), int(y + v)), (0, 255, 0), 1)
        cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)
    return frame

def detect_obstacles(flow_vectors, img_width, obstacle_threshold=2.0):
    """
    Slide a window along the x-axis to detect obstacles.
    The 'obstacle_threshold' is critical: if the mean magnitude of the flow vectors
    within the window is greater than this threshold, the window is marked as an obstacle.
    """
    window_width = img_width // 6
    step_size = 4
    detections = []
    for x_start in range(0, img_width - window_width + 1, step_size):
        x_end = x_start + window_width
        mags = [math.sqrt(u*u + v*v) for (x, y, u, v) in flow_vectors if x_start <= x < x_end]
        if mags and np.mean(mags) > obstacle_threshold:  # Threshold set to 4.0
            detections.append((x_start, x_end))
    # Merge overlapping regions
    merged = []
    if detections:
        current = detections[0]
        for region in detections[1:]:
            if region[0] <= current[1]:
                current = (current[0], region[1])
            else:
                merged.append(current)
                current = region
        merged.append(current)
    return merged

def compute_free_regions(obstacle_regions, img_width):
    """
    Compute free regions along the x-axis as the complement of the obstacle regions.
    """
    free_regions = []
    current = 0
    for (x_start, x_end) in sorted(obstacle_regions, key=lambda r: r[0]):
        if current < x_start:
            free_regions.append((current, x_start))
        current = x_end
    if current < img_width:
        free_regions.append((current, img_width))
    return free_regions

def main():
    # Open the PC webcam (device index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return

    # Capture the first frame
    ret, prev_frame = cap.read()
    if not ret:
        return

    # Optionally rotate the frame if needed
    # prev_frame = rotate_image(prev_frame)
    height, width, _ = prev_frame.shape

    # Initialize the video writer to save output video (optional)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    output_video = cv2.VideoWriter('output.avi', fourcc, 10, (width, height))
    obstacle_threshold = 4.0  # Critical threshold for obstacle detection

    while True:
        ret, next_frame = cap.read()
        if not ret:
            break

        # next_frame = rotate_image(next_frame)

        # Compute optical flow between the previous and current frame
        flow_vectors = compute_optical_flow_lk(prev_frame, next_frame, window_size=5, step=16)
        vis_frame = prev_frame.copy()
        vis_frame = draw_optical_flow(flow_vectors, vis_frame)

        # Detect obstacles based on flow magnitude using the defined threshold
        obstacle_regions = detect_obstacles(flow_vectors, width, obstacle_threshold)
        free_regions = compute_free_regions(obstacle_regions, width)

        # Fill free regions with a blue overlay
        for (x_start, x_end) in free_regions:
            cv2.rectangle(vis_frame, (x_start, 0), (x_end, height), (255, 0, 0), thickness=cv2.FILLED)
        # Draw red boundaries on detected obstacle regions
        for (x_start, x_end) in obstacle_regions:
            cv2.rectangle(vis_frame, (x_start, 0), (x_end, height), (0, 0, 255), 2)

        output_video.write(vis_frame)
        cv2.imshow("Optical Flow and Obstacles", vis_frame)

        # Update previous frame for the next iteration
        prev_frame = next_frame.copy()

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    output_video.release()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
