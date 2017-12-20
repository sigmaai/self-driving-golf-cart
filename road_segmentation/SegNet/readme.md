### SegNet-Basic:
---

What is Segnet?

* Deep Convolutional Encoder-Decoder Architecture for Semantic Pixel-wise Image Segmentation

 **Segnet** = **(Encoder + Decoder)** +  **Pixel-Wise Classification** layer

##### *[SegNet: A Deep Convolutional Encoder-Decoder Architecture for Image Segmentation (Vijay Badrinarayanan, Alex Kendall, Roberto Cipolla, Senior Member, IEEE) arXiv:1511.00561v3](https://arxiv.org/abs/1511.00561)*


What is SegNet-Basic?

* *"In order to analyse SegNet and compare its performance with FCN  (decoder  variants)  we  use  a  smaller  version  of  SegNet, termed SegNet-Basic ,  which  ha  4  encoders  and  4  decoders. All the encoders in SegNet-Basic perform max-pooling and subsampling and the corresponding decoders upsample its input using the  received  max-pooling  indices."*

Basically it's a mini-segnet to experiment / test the architecure with convnets, such as FCN.


 -----

### Steps To Run The Model:
---

1. Run `python model-basic.py` to create `segNet_basic_model` for keras to use.
	
	* `model-basic.py` contains the architecure.

2. 



### Dataset:
---

1. In a different directory run this to download the [dataset from original Implementation](https://github.com/alexgkendall/SegNet-Tutorial).
	* `git clone git@github.com:alexgkendall/SegNet-Tutorial.git`
	* copy the `/CamVid` to here, or change the `DataPath` in `data_loader.py` to the above directory
2. The run `python data_loader.py` to generate these two files:
	
	* `/data/train_data.npz/` and `/data/train_label.npz`
	* This will make it easy to process the model over and over, rather than waiting the data to be loaded into memory.



----


### To Do:
----

	[x] SegNet-Basic
	[ ] SegNet
	[x] Test Accuracy
	[ ] Requirements


### Segnet-Basic Road Scene Results:
--------

* Train / Test:

```
	Train on 367 samples, validate on 233 samples
	Epoch 101/102
	366/367 [============================>.] 
	- ETA: 0s - loss: 0.3835 - acc: 0.8737Epoch 00000: val_acc improved from -inf to 0.76367, saving model to weights.best.hdf5
	367/367 [==============================] 
	- 231s - loss: 0.3832 - acc: 0.8738 - val_loss: 0.7655 - val_acc: 0.7637
	Epoch 102/102
	366/367 [============================>.] 
	- ETA: 0s - loss: 0.3589 - acc: 0.8809Epoch 00001: val_acc did not improve
	367/367 [==============================] 
	- 231s - loss: 0.3586 - acc: 0.8810 - val_loss: 2.4447 - val_acc: 0.4478
```


* Evaluation:


	`acc: 85.47%`


    ![img1](./imgs_results/segmented_road_scene.png)


	![img2](./imgs_results/real_road_scene.png)
