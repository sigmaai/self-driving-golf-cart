#
# MIT License

# Copyright (c) 2017 Yongyang Nie, Michael Meng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# do not run this file from this directory
# run it from the top directory of the workspace

from object_detector import ObjectDetector
import cv2
import numpy as np
from imutils.video import VideoStream


def read_cam():

    detector = ObjectDetector()

    # OpenCV main loop
    vid_center = VideoStream(src=0).start()

    while True:

        windowName = "car detection"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 416, 416)
        cv2.moveWindow(windowName, 0, 0)
        cv2.setWindowTitle(windowName, "car detection")

        while True:

            # -----------------------------------------------------
            # Check to see if the user closed the window
            if cv2.getWindowProperty(windowName, 0) < 0:
                # This will fail if the user closed the window; get printed to the console
                break
            image = vid_center.read()

            # -----------------------------------------------------
            # run the network and detection

            result = detector.detect_object(image, visualize=True)

            displayBuf = np.array(result)

            # show the stuff
            # -----------------------------------------------------

            cv2.imshow(windowName, displayBuf)
            key = cv2.waitKey(10)
            if key == 27:  # ESC key
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    read_cam()
