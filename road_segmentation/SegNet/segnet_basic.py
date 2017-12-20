from __future__ import absolute_import
from __future__ import print_function
import model

# load the data
# train_data = np.load('./data/train_data.npy')
# train_label = np.load('./data/train_label.npy')
#
# test_data = np.load('./data/test_data.npy')
# test_label = np.load('./data/test_label.npy')

# load the model:
segnet_basic = model.segnet(nb_classes=2, input_height=480, input_width=640)
print(segnet_basic.summary)

nb_epoch = 100
batch_size = 6

# Fit the model
# history = segnet_basic.fit(train_data, train_label,
#                            batch_size=batch_size,
#                            nb_epoch=nb_epoch, verbose=1,
#                            class_weight=class_weighting ,
#                            validation_data=(test_data, test_label), shuffle=True) # validation_split=0.33
#
# # This save the trained model weights to this file with number of epochs
# segnet_basic.save_weights('weights/model_weight_{}.hdf5'.format(nb_epoch))

