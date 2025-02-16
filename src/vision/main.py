"""
This module provides a vision-based model for detecting and processing persons in video 
frames using YOLO and BLIP models.

ML Models use: YOLOv11, BLIP, KeyBERT
"""

from ultralytics import YOLO
from transformers import AutoProcessor, AutoModel
from transformers import BlipProcessor, BlipForConditionalGeneration
from PIL import Image
from keybert import KeyBERT
import cv2
import os

_WEBCAM = 0
_SAVE_DIR = "output"
BATCH_SIZE = 4
FRAME_SKIP = 3

class Model:
    def __init__(self):
        self.yolo_model = YOLO("yolo11n.pt")
        self.blip_processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
        self.blip_model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")
        self.keybert_model = KeyBERT('distilbert-base-nli-mean-tokens')
        
        self.cap = cv2.VideoCapture(_WEBCAM)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        
        os.makedirs(_SAVE_DIR, exist_ok=True)
        
        self.frame_count = 0
        
        self.image_batch = []
        self.cropped_persons = []
        self.person_directions = []

    def predict(self, frame=_WEBCAM):
        self.person_directions = []
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            self.frame_count += 1
            
            if self.frame_count % FRAME_SKIP == 0:
                results = self.yolo_model.predict(
                    source=frame,
                    conf=0.5,
                    iou=0.5,
                    classes=[0],  # only detect Person ClassName
                )
                
                if len(results[0].boxes) > 0:
                    for i, box in enumerate(results[0].boxes):
                        direction = self.get_box_direction(frame, box)
                        self.person_directions.append(direction)  # Store direction

                        coords = box.xyxy.cpu().detach().numpy().squeeze().tolist()
                        x1, y1, x2, y2 = map(int, coords)
                        cropped = frame[y1:y2, x1:x2]
                        self.cropped_persons.append(cropped)
                        
                        # Process in batches
                        if len(self.cropped_persons) >= BATCH_SIZE:
                            self.process_person_batch()
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if self.cropped_persons:
            self.process_person_batch()

        self.cleanup()

    def process_person_batch(self):
        """Process a batch of detected persons"""
        batch_texts = []
        directions = []
        
        for i, cropped in enumerate(self.cropped_persons):
            filename = os.path.join(_SAVE_DIR, f"person_{i}.jpg")
            cv2.imwrite(filename, cropped)

            direction = self.person_directions[i] if i < len(self.person_directions) else ""
            directions.append(direction)
            
            image = Image.fromarray(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB))
            self.image_batch.append(image)
        
        if self.image_batch:
            inputs = self.blip_processor(images=self.image_batch, return_tensors="pt", padding=True)
            outputs = self.blip_model.generate(**inputs)
            
            for i, output in enumerate(outputs):
                text = self.blip_processor.decode(output, skip_special_tokens=True)

                direction = directions[i]
                combined_text = f"Detected: {text} - Location: {direction}"
                batch_texts.append(combined_text)
                print(combined_text)
                
        self.image_batch = []
        self.cropped_persons = []
        self.person_directions = []
    
    def get_box_direction(self, frame, box):
        frame_height, frame_width = frame.shape[:2]
        coords = box.xyxy.cpu().detach().numpy().squeeze().tolist()
        x1, y1, x2, y2 = map(int, coords)
        
        # Calculate center points
        box_center_x = (x1 + x2) / 2
        box_center_y = (y1 + y2) / 2
        
        # Calculate relative position
        rel_x = (box_center_x - frame_width / 2) / (frame_width / 2)
        rel_y = (box_center_y - frame_height / 2) / (frame_height / 2)
        
        direction = []
        if rel_y < -0.2:
            direction.append("above")
        elif rel_y > 0.2:
            direction.append("below")
        
        if rel_x < -0.2:
            direction.append("to the left")
        elif rel_x > 0.2:
            direction.append("to the right")
        else:
            direction.append("in center")
            
        return " ".join(direction)

    def cleanup(self):
        """Cleanup resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        
        for file in os.listdir(_SAVE_DIR):
            os.remove(os.path.join(_SAVE_DIR, file))
    
if __name__ == "__main__":
    model = Model()
    model.predict()
