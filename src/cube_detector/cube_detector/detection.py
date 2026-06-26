import cv2
import numpy as np

CONF_THRESHOLD = 0.6
NMS_THRESHOLD = 0.3
IMAGE_SIZE = (320, 320)

def run_detection(frame):
    return cv2.dnn.blobFromImage(
        frame,
        scalefactor=1 / 255.0,
        size=IMAGE_SIZE,
        swapRB=True,
        crop=False
    )

def postprocess(outputs, frame_shape):
    h, w = frame_shape[:2]

    detections = []
    predictions = outputs[0].T

    boxes = []
    scores = []
    class_ids = []

    for pred in predictions:
        class_id = int(np.argmax(pred[4:]))
        confidence = float(pred[4:][class_id])

        if confidence < CONF_THRESHOLD:
            continue

        x_center, y_center, bw, bh = pred[:4]

        x1 = int((x_center - bw / 2) * w / IMAGE_SIZE[0])
        y1 = int((y_center - bh / 2) * h / IMAGE_SIZE[1])
        x2 = int((x_center + bw / 2) * w / IMAGE_SIZE[0])
        y2 = int((y_center + bh / 2) * h / IMAGE_SIZE[1])

        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(confidence)
        class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, NMS_THRESHOLD)

    if len(indices) > 0:
        i = indices.flatten()[0]
        x, y, bw, bh = boxes[i]

        detections.append((x, y, x + bw, y + bh, scores[i], class_ids[i]))

    return detections

def draw_detections(frame, detections):
    for x1, y1, x2, y2, conf, class_id in detections:

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            4
        )

        cv2.putText(
            frame,
            f"Cube: {conf:.2f}",
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 0),
            3
        )

    return frame