import cv2


def create_tracker():
    # В разных версиях OpenCV CSRT находится в разных местах
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()

    if hasattr(cv2, "legacy"):
        return cv2.legacy.TrackerCSRT_create()

    raise RuntimeError(
        "CSRT-трекер не найден.\n"
        "Установите: pip install opencv-contrib-python"
    )


camera = cv2.VideoCapture(0)

tracker = None

while True:
    ok, frame = camera.read()

    if not ok:
        break

    # Если трекер уже запущен — обновляем положение объекта
    if tracker is not None:
        found, box = tracker.update(frame)

        if found:
            x, y, w, h = map(int, box)

            # Рамка вокруг объекта
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            # Центр объекта
            cx = x + w // 2
            cy = y + h // 2

            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            cv2.putText(
                frame,
                f"Center: {cx}, {cy}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        else:
            cv2.putText(
                frame,
                "Object lost",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

    cv2.putText(
        frame,
        "S - select object   R - reset   Q - exit",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.imshow("CSRT Tracker", frame)

    key = cv2.waitKey(1) & 0xFF

    # Выход
    if key == ord("q") or key == 27:
        break

    # Сброс трекера
    if key == ord("r"):
        tracker = None

    # Выбрать объект.
    if key == ord("s"):
        box = cv2.selectROI(
            "Select object",
            frame,
            fromCenter=False,
            showCrosshair=True
        )

        cv2.destroyWindow("Select object")

        if box[2] > 0 and box[3] > 0:
            tracker = create_tracker()
            tracker.init(frame, box)


camera.release()
cv2.destroyAllWindows()
