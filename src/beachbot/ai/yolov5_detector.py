from .debrisdetector import DebrisDetector

import numpy as np
import cv2 as cv


class Yolo5Detector(DebrisDetector):
    def __init__(self, model_file=None) -> None:
        super().__init__(model_file)

    def apply_model(self, inputs):
        raise NotImplementedError

    def _crop_and_scale_image(self, image):
        h = image.shape[0]
        w = image.shape[1]
        intended_x = self.img_width
        intended_y = self.img_height
        ratio_w = intended_x / w
        ratio_h = intended_y / h
        maxratio = max(ratio_w, ratio_h)
        image = cv.resize(image, (int(w * maxratio), int(h * maxratio)))
        crop_w = image.shape[1] - intended_x
        crop_h = image.shape[0] - intended_y
        if crop_h > 0:
            image = image[(crop_h // 2) : (crop_h // 2) + intended_y]
        if crop_w > 0:
            image = image[:, (crop_w // 2) : (crop_w // 2) + intended_x]
        return image

    # Do somhow the inverse mapping of box coordinates to compensate _crop_and_scale_image
    def _map_resuts_to_input_image(self, boxes, image, units_percent=True):
        h = image.shape[0]
        w = image.shape[1]
        intended_x = self.img_width
        intended_y = self.img_height
        ratio_w = intended_x / w
        ratio_h = intended_y / h
        maxratio = max(ratio_w, ratio_h)
        w = int(w * maxratio)
        h = int(h * maxratio)

        crop_w = w - intended_x
        crop_h = h - intended_y

        for b in boxes:
            # box format is: [left, top, width, height]
            # add offsets of box left & top to match cropped image from _crop_and_scale:
            b[0] += crop_w // 2
            b[1] += crop_h // 2

            if units_percent:
                b[0] /= w
                b[1] /= h
                b[2] /= w
                b[3] /= h
            else:
                # pixel coordinates, round box coordinates to int
                b[0] = round(b[0])
                b[1] = round(b[1])
                b[2] = round(b[2])
                b[3] = round(b[3])

    def _wrap_detection(self, output_data, confidence_threshold=0.2):
        class_ids = []
        confidences = []
        boxes = []
        rows = output_data.shape[0]

        for r in range(rows):
            row = output_data[r]
            box_confidence = row[4]

            if box_confidence >= confidence_threshold:

                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)

                # for YOLO, confidence is calculated by box_confidence*class_csonfidence, if we exceed, we add it to the results:
                if classes_scores[class_id] * box_confidence > confidence_threshold:
                    confidences.append(classes_scores[class_id] * box_confidence)
                    class_ids.append(class_id)

                    x, y, w, h = (
                        row[0].item(),
                        row[1].item(),
                        row[2].item(),
                        row[3].item(),
                    )
                    left = x - 0.5 * w
                    top = y - 0.5 * h
                    width = w
                    height = h

                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv.dnn.NMSBoxes(boxes, confidences, confidence_threshold, 0.45)

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes
