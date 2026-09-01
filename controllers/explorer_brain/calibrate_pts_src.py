import cv2
import numpy as np

IMG_PATH = "raw_frame.png"

img = cv2.imread(IMG_PATH)
if img is None:
    raise FileNotFoundError(f"Could not find {IMG_PATH} - see instructions at top of this file.")

clone = img.copy()
points = []
labels = ["bottom-left", "bottom-right", "top-right", "top-left"]


def redraw():
    display = clone.copy()
    for i, p in enumerate(points):
        cv2.circle(display, p, 5, (0, 0, 255), -1)
        cv2.putText(display, labels[i], (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    if len(points) >= 2:
        for i in range(len(points) - 1):
            cv2.line(display, points[i], points[i + 1], (0, 255, 0), 1)
    cv2.imshow("Click 4 ground points (bl, br, tr, tl)", display)


def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))
        print(f"{labels[len(points) - 1]}: ({x}, {y})")
        redraw()
        if len(points) == 4:
            pts = np.float32(points)
            print("\nPaste this into option_a_recalibrated.py:\n")
            print("ptsSrc = np.float32([")
            for i, p in enumerate(pts):
                print(f"    [{p[0]:.1f}, {p[1]:.1f}],  # {labels[i]}")
            print("])")


cv2.namedWindow("Click 4 ground points (bl, br, tr, tl)")
cv2.setMouseCallback("Click 4 ground points (bl, br, tr, tl)", on_click)
redraw()

while True:
    k = cv2.waitKey(1) & 0xFF
    if k == ord('r'):
        points = []
        redraw()
    elif k == ord('q'):
        break

cv2.destroyAllWindows()