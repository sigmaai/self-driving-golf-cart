Advanced Lane Finding
---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/undistort_output.png "Undistorted"
[image2]: ./output_images/undistort_car.png "Road Undistorted"
[image3]: ./output_images/color.png "Color Lane Detection"
[image4]: ./output_images/gradient.png "Gradient Lane Detection"
[image5]: ./output_images/combined.png "Combined lane Detection"
[image6]: ./output_images/straight.png "Straight Warped"
[image7]: ./output_images/curved.png "Curved Warped"
[image8]: ./output_images/histogram.png "histogram"
[image9]: ./output_images/fit_curve.png "Fit Poly Curve"
[image10]: ./output_images/plot.png "Plot"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is in the `main.ipynb`, under the title "Camera Calibration" 

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

Here is an example of a distortion-corrected image.
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (see "Edge Detection" section in the `main.ipynb`).  Here are the output of the warped test images.

Color thresholding:
![alt text][image3]

Gradient thresholding:
![alt text][image4]

Combining these two:
![alt text][image5]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is under the section "Perspective Transformation" in `main.ipynb` . It maps the source (`src`) to destination (`dst`) points.

Here are the points I chose

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 190, 720      | 200, 720        | 
| 599, 445      | 200, 0    |
| 697, 445      | 970, 0      |
| 1124, 720     | 970, 720        |

Here are the warped images for the straight lines:
![alt text][image6]

For the test images:
![alt text][image7]


#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

In fit curve section, I have two methods for fitting the curve to the detected image.One uses the sliding window method, with the initial position found using a histogram for the lower part of the images.

Here is an image of the histogram. The peaks left and right to the center are what I used for finding the bottom windows.
![alt text][image8]

Here is one result using the sliding window.

![alt text][image9]

The other one is for videos, which simply searched for pixels near the previous fitted lines.

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in the "Curvature" section in `main.ipynb`.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in the "Pipeline for Image" section in `main.ipynb` 

Here is an example of my result on test images:

![alt text][image10]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./result.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The core of this project for me is the section of detection lane lines using color and gradient threshold. The images can be very sensitive in response to the threshold. 

My pipeline is robust enough to identify lanes even in very bright or dark sections, as I implemented an inverse relation between the threshold for saturation and the brightness. However, It is likely to miss identify the left-most line as a lane. This does not occur in this project

I could use relative length to the image height and width to specify the source and destiny points, so that the functions can work on different sized videos as well.