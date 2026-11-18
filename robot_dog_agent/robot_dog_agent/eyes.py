"""Look through the camera, ask the model what is there, remember where (spec 2026-11-18, 4.1). Plain Python.

snap() -> vision.Snapshot or None (camera_feed.CameraFeed.snapshot in the window); ask_image(prompt, jpeg) -> the
model's text (gemini.GeminiClient.ask_image); memory: an objects.ObjectMemory.
"""
from robot_dog_agent import vision


class Eyes:
    def __init__(self, snap, ask_image, memory):
        self.snap, self.ask_image, self.memory = snap, ask_image, memory

    def look(self, target=None):
        """The things seen in one picture (only `target` when given), each also added to the memory.
        The robot must be standing still. RuntimeError without a picture; the model's errors pass through."""
        picture = self.snap()
        if picture is None:
            raise RuntimeError('no camera image yet')
        jpeg = vision.encode_jpeg(picture.rgb, picture.width, picture.height)
        answer = self.ask_image(vision.detection_prompt(target), jpeg)
        seen = vision.locate(vision.parse_detections(answer), picture)
        for thing in seen:
            self.memory.add(thing)
        return seen
