# Semantic Segmentation

🚧 This is still under development 🏗

Semantic segmentation has become an important component of self-driving vehicles. It allows the car to understand the surroundings by classifying every pixel of the input image.

![](./media/image-1.png)

## Running...
To run inference on the pre-trained models, please use `segmentor.py`. 

	from segmentor import Segmentor
	seg = Segmentor()
	cls_output, img_viz = seg.semantic_segmentation(image=image, visualization=True)

`cls_output` is the pixel-wise classification result for all the categories. `img_viz` is a RGB image generated based on `cls_output`

The best way to run some actual test is using `test.py`. You must specify the image path by changing the `path` variable. 

The pre-trained weights are stored in the `./weights` directory.

## Training

`train.py` is the training script. `utils.py` contains all the categories (classes). You can modify them based on your dataset. 

*training doesn't work very well.* I trying to fix the issue. 

## Models

The goal is to achieve real time semantic segmentation. Therefore, choosing & developing the appropriate model is critical. 
I experimented with UNet and ENet, but realized drawbacks to both of these architectures. I eventually landed on ICNet, which
stands for image cascade network

![](./media/model_overview.png)

Here is a simple benchmark comparison between ICNet and other popular semantic segmentation models. These images and visualizations
are from the original ICNet paper, which can be found [here](https://arxiv.org/abs/1704.08545).  

<center>
<img src="./media/model_comparison.png" alt="image" width="500">
</center>

## About

This project is created for the self-driving golf cart project that I have been working on. For more information on that, please refer to the [Github page](https://github.com/xmeng17/self-driving-golf-cart), or my [website](https://neilnie.com/the-self-driving-golf-cart-project/).

If you have questions, comments or concerns, please contact me at [contact@neilnie.com](mailto:contact@neilnie.com). 

## Others

#### Details about trained models
`./weights/new-enet-5.h5`: 512x512x26

`./weights/enet-c-v1-2.h5`: 640x320x16

`./weights/enet-c-v2-1.h5`: 1024x512x16

The channel value correspond to the number of classes.