# Training Notes

## Single Frame Training
### DenseNet
After 15x1000 epoch training, the network seems to have converged, with a loss of ~5. More training even brought the loss up to ~6. Testing with the visualizer shows the network doesn't know to speed up nor slow down. More training doesn't seems to improve the performance. 
DenseNet has around 120k parameters.

### NvidiaNet
This network architecture is inpsired by the CNN in the Nvidia self driving paper. The training and inference time of this network is much faster than DenseNet and sVGG. After 50x1000 epochs, the network converges with the loss around 1. The result is encouraging comparing to DenseNet. Visualization shows the network knows to slow down or speed up in situation. However, better results can be acheived.
NvidiaNet has around 3m parameters

10,000 steps, 15 epochs <br>
150000 steps

### sVGGNet
Trained for: 

2000 steps, 10 epochs <br>
1000 steps 15 epochs <br>
1000 x 35 epochs = 35000 steps

loss: ~2.0

### CommaNet
2000 steps, ~25 epochs <br>
loss ~2.0
