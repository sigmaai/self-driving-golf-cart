#
# SamplingRNNCell
# (c) Yongyang Nie, 2018
# Contact: contact@neilnie.com
#

import tensorflow as tf


class SamplingRNNCell(tf.nn.rnn_cell.RNNCell):

    """Simple sampling RNN cell."""

    def __init__(self, num_outputs, use_ground_truth, internal_cell, keep_prob):
        """
        if use_ground_truth then don't sample
        """
        self._num_outputs = num_outputs
        self._use_ground_truth = use_ground_truth  # boolean
        self._internal_cell = internal_cell  # may be LSTM or GRU or anything
        self.__keep_prob = keep_prob

    @property
    def state_size(self):
        return self._num_outputs, self._internal_cell.state_size  # previous output and bottleneck state

    @property
    def output_size(self):
        # steering angle, torque, vehicle speed
        return self._num_outputs

    def __call__(self, inputs, state, scope=None):
        (visual_feats, current_ground_truth) = inputs
        prev_output, prev_state_internal = state
        context = tf.concat([prev_output, visual_feats], 1)
        new_output_internal, new_state_internal = self._internal_cell(context, prev_state_internal)  # here the internal cell (e.g. LSTM) is called
        new_output = tf.contrib.layers.fully_connected(inputs=tf.concat([new_output_internal, prev_output, visual_feats], 1),
                                                       num_outputs=self._num_outputs,
                                                       activation_fn=None,
                                                       scope="OutputProjection")

        # if self._use_ground_truth == True, we pass the ground truth as the state;
        # otherwise, we use the model's predictions
        return new_output, (current_ground_truth if self._use_ground_truth else new_output, new_state_internal)
