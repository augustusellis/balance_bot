from threading import Thread
import cv2
import time

class Webcam:
    # Class which contains features for grabbing frames from a webcam.
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()

        self.stopped = False # Used to indicate if the thread should be stopped

        # Define variables used to calculate Frames per Second:
        self.start_time = None
        self.end_time = None
        self.num_frames = 0

    def start(self):
        # Run update method in separate thread.
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Indefinitely loop until thread is stopped
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

    def fps_start(self):
        self.num_frames = 0
        self.start_time = time.time()
        return self

    def fps_stop(self):
        self.end_time = time.time()

    def fps_update(self):
        self.num_frames += 1

    def fps(self):
        return self.num_frames/(self.end_time-self.start_time)
