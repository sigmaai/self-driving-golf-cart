#
# AutoPilot
#

import tensorflow as tf
import scipy.misc
import steering.ap_model as model


class AutoPilot:

    def __init__(self):
        sess = tf.InteractiveSession()
        saver = tf.train.Saver()
        saver.restore(sess, "steering/weights/AutoPilot/model.ckpt")

    def predict(self, frame):

        image = scipy.misc.imresize(frame, [66, 200]) / 255.0
        rad = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0})[0][0]

        return rad
