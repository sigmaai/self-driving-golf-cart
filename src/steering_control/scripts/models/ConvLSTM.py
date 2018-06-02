#
# ConvLSTM training file
# Experimentation code
# (c) Yongyang Nie, 2018, All Rights Reserved
# Contact: contact@neilnie.com
#

import numpy as np
import tensorflow as tf
import os
from SamplingRNNCell import SamplingRNNCell
from utils import BatchGenerator
import configs
import utils
import helper as helper

slim = tf.contrib.slim
global_train_step = 0
global_valid_step = 0


class ConvLSTM(object):

    # this is the input part
    # conv lstm

    def apply_vision_simple(self, image, keep_prob, batch_size, seq_len, scope=None, reuse=None):

        video = tf.reshape(image, shape=[batch_size, configs.LEFT_CONTEXT + seq_len, configs.HEIGHT, configs.WIDTH,
                                         configs.CHANNELS])

        with tf.variable_scope(scope, 'Vision', [image], reuse=reuse):
            net = slim.convolution(video, num_outputs=64, kernel_size=[3, 12, 12], stride=[1, 6, 6], padding="VALID")
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            aux1 = slim.fully_connected(tf.reshape(net[:, -seq_len:, :, :, :], [batch_size, seq_len, -1]), 128,
                                        activation_fn=None)

            net = slim.convolution(net, num_outputs=64, kernel_size=[2, 5, 5], stride=[1, 2, 2], padding="VALID")
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            aux2 = slim.fully_connected(tf.reshape(net[:, -seq_len:, :, :, :], [batch_size, seq_len, -1]), 128,
                                        activation_fn=None)

            net = slim.convolution(net, num_outputs=64, kernel_size=[2, 5, 5], stride=[1, 1, 1], padding="VALID")
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            aux3 = slim.fully_connected(tf.reshape(net[:, -seq_len:, :, :, :], [batch_size, seq_len, -1]), 128,
                                        activation_fn=None)

            net = slim.convolution(net, num_outputs=64, kernel_size=[2, 5, 5], stride=[1, 1, 1], padding="VALID")
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            # at this point the tensor 'net' is of shape batch_size x seq_len x ...
            aux4 = slim.fully_connected(tf.reshape(net, [batch_size, seq_len, -1]), 128, activation_fn=None)

            net = slim.fully_connected(tf.reshape(net, [batch_size, seq_len, -1]), 1024, activation_fn=tf.nn.relu)
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            net = slim.fully_connected(net, 512, activation_fn=tf.nn.relu)
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            net = slim.fully_connected(net, 256, activation_fn=tf.nn.relu)
            net = tf.nn.dropout(x=net, keep_prob=keep_prob)
            net = slim.fully_connected(net, 128, activation_fn=None)

            return self.layer_norm(tf.nn.elu(net + aux1 + aux2 + aux3 + aux4))  # aux[1-4] are residual connections (shortcuts)

    def do_epoch(self, session, sequences, mode):

        global global_train_step, global_valid_step
        test_predictions = {}
        valid_predictions = {}
        batch_generator = BatchGenerator(sequence=sequences, seq_len=configs.SEQ_LEN, batch_size=configs.BATCH_SIZE)
        total_num_steps = int(1 + (batch_generator.indices[1] - 1) / configs.SEQ_LEN)
        controller_final_state_gt_cur, controller_final_state_autoregressive_cur = None, None
        acc_loss = np.float128(0.0)

        for step in range(total_num_steps):

            feed_inputs, feed_targets = batch_generator.next()
            feed_dict = {self.inputs: feed_inputs, self.targets: feed_targets}
            if controller_final_state_autoregressive_cur is not None:
                feed_dict.update({self.controller_initial_state_autoregressive: controller_final_state_autoregressive_cur})
            if controller_final_state_gt_cur is not None:
                feed_dict.update({self.controller_final_state_gt: controller_final_state_gt_cur})

            if mode == "train":
                feed_dict.update({self.keep_prob: configs.KEEP_PROB_TRAIN})
                summary, _, loss, controller_final_state_gt_cur, controller_final_state_autoregressive_cur = session.run([self.summaries, self.optimizer, self.mse_autoregressive_steering, self.controller_final_state_gt, self.controller_final_state_autoregressive],
                                                                                                                         feed_dict=feed_dict)
                self.train_writer.add_summary(summary, global_train_step)
                global_train_step += 1

            elif mode == "valid":
                model_predictions, summary, loss, controller_final_state_autoregressive_cur = session.run([self.steering_predictions, self.summaries, self.mse_autoregressive_steering, self.controller_final_state_autoregressive],
                                                                                                          feed_dict=feed_dict)
                self.valid_writer.add_summary(summary, global_valid_step)
                global_valid_step += 1
                feed_inputs = feed_inputs[:, configs.LEFT_CONTEXT:].flatten()
                steering_targets = feed_targets[:, :, 0].flatten()
                model_predictions = model_predictions.flatten()
                stats = np.stack([steering_targets, model_predictions, (steering_targets - model_predictions) ** 2])
                for i, img in enumerate(feed_inputs):
                    valid_predictions[img] = stats[:, i]

            elif mode == "test":
                model_predictions, controller_final_state_autoregressive_cur = session.run([self.steering_predictions, self.controller_final_state_autoregressive], feed_dict=feed_dict)
                feed_inputs = feed_inputs[:, configs.LEFT_CONTEXT:].flatten()
                model_predictions = model_predictions.flatten()
                for i, img in enumerate(feed_inputs):
                    test_predictions[img] = model_predictions[i]

            if mode != "test":
                acc_loss += loss
                print(str(step + 1) + "/" + str(total_num_steps) + " " + "loss: " + str(np.sqrt(acc_loss / (step + 1))))

        if mode != "test":
            return np.sqrt(acc_loss / total_num_steps), valid_predictions
        else:
            return None, test_predictions

    def train(self, graph, saver):

        # the main training portion
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=1.0)

        checkpoint_dir = os.getcwd() + "/v3"

        best_validation_score = None

        with tf.Session(graph=graph, config=tf.ConfigProto(gpu_options=gpu_options)) as session:

            session.run(tf.initialize_all_variables())
            print('Initialized')
            ckpt = tf.train.latest_checkpoint(checkpoint_dir)

            if ckpt:
                print("Restoring from", ckpt)
                saver.restore(sess=session, save_path=ckpt)

            # training for a number of epochs
            for epoch in range(configs.NUM_EPOCHS):

                print("Starting epoch %d" % epoch)
                print("Validation:")
                valid_score, valid_predictions = self.do_epoch(session=session, sequences=self.valid_seq, mode="valid")

                if best_validation_score is None:
                    best_validation_score = valid_score

                if valid_score < best_validation_score:
                    saver.save(session, 'v3/checkpoint-sdc-ch2')
                    best_validation_score = valid_score
                    print('\r', "SAVED at epoch %d" % epoch)
                    with open("v3/valid-predictions-epoch%d" % epoch, "w") as out:
                        result = np.float128(0.0)
                        for img, stats in valid_predictions.items():
                            result += stats[-1]

                    print("Validation unnormalized RMSE:", np.sqrt(result / len(valid_predictions)))

                    with open("v3/test-predictions-epoch%d" % epoch, "w") as out:
                        _, test_predictions = self.do_epoch(session=session, sequences=self.test_seq, mode="test")
                        # print("frame_id,steering_angle")
                        for img, pred in test_predictions.items():
                            img = img.replace("challenge_2/Test-final/center/", "")
                            # print("%s,%f" % (img, pred))

                if epoch != configs.NUM_EPOCHS - 1:
                    print("Training")
                    self.do_epoch(session=session, sequences=self.train_seq, mode="train")

    def build_graph(self):

        #
        # This method builds the TensorFlow graph, and also
        # Set several critical instance variables:
        #   self.learning_rate
        #   self.keep_prob
        #   self.inputs
        #   self.targets
        #   self.controller_initial_state_autoregressive
        #   self.controller_final_state_gt
        #   self.controller_final_state_autoregressive
        #   self.mse_autoregressive_steering
        #
        # Parameters:
        #   None
        # Return:
        #   graph, saver, optimizer, steering_predictions,
        #   summaries, train_writer, valid_writer

        graph = tf.Graph()

        with graph.as_default():
            # inputs
            learning_rate = tf.placeholder_with_default(input=1e-4, shape=())
            self.keep_prob = tf.placeholder_with_default(input=1.0, shape=())

            self.inputs = tf.placeholder(shape=(configs.BATCH_SIZE, configs.LEFT_CONTEXT + configs.SEQ_LEN),
                                    dtype=tf.string)  # pathes to png files from the central camera
            self.targets = tf.placeholder(shape=(configs.BATCH_SIZE, configs.SEQ_LEN, configs.OUTPUT_DIM),
                                     dtype=tf.float32)  # seq_len x batch_size x OUTPUT_DIM
            targets_normalized = (self.targets - self.mean) / self.std

            input_images = tf.stack([tf.image.decode_png(tf.read_file(x))
                                     for x in
                                     tf.unstack(tf.reshape(self.inputs, shape=[(configs.LEFT_CONTEXT + configs.SEQ_LEN) * configs.BATCH_SIZE]))])
            input_images = -1.0 + 2.0 * tf.cast(input_images, tf.float32) / 255.0
            input_images.set_shape([(configs.LEFT_CONTEXT + configs.SEQ_LEN) * configs.BATCH_SIZE, configs.HEIGHT, configs.WIDTH, configs.CHANNELS])
            visual_conditions_reshaped = self.apply_vision_simple(image=input_images, keep_prob=self.keep_prob,
                                                             batch_size=configs.BATCH_SIZE, seq_len=configs.SEQ_LEN)
            visual_conditions = tf.reshape(visual_conditions_reshaped, [configs.BATCH_SIZE, configs.SEQ_LEN, -1])
            visual_conditions = tf.nn.dropout(x=visual_conditions, keep_prob=self.keep_prob)

            rnn_inputs_with_ground_truth = (visual_conditions, targets_normalized)
            rnn_inputs_autoregressive = (
            visual_conditions, tf.zeros(shape=(configs.BATCH_SIZE, configs.SEQ_LEN, configs.OUTPUT_DIM), dtype=tf.float32))

            internal_cell = tf.nn.rnn_cell.LSTMCell(num_units=configs.RNN_SIZE, num_proj=configs.RNN_PROJ)
            cell_with_ground_truth = SamplingRNNCell(num_outputs=configs.OUTPUT_DIM, use_ground_truth=True,
                                                     internal_cell=internal_cell, keep_prob=self.keep_prob)
            cell_autoregressive = SamplingRNNCell(num_outputs=configs.OUTPUT_DIM, use_ground_truth=False,
                                                  internal_cell=internal_cell, keep_prob=self.keep_prob)

            controller_initial_state_variables = helper.get_initial_state(cell_autoregressive.state_size)
            self.controller_initial_state_autoregressive = helper.deep_copy_initial_state(controller_initial_state_variables)
            controller_initial_state_gt = helper.deep_copy_initial_state(controller_initial_state_variables)

            with tf.variable_scope("predictor"):
                out_gt, self.controller_final_state_gt = tf.nn.dynamic_rnn(cell=cell_with_ground_truth,
                                                                      inputs=rnn_inputs_with_ground_truth,
                                                                      sequence_length=[configs.SEQ_LEN] * configs.BATCH_SIZE,
                                                                      initial_state=controller_initial_state_gt,
                                                                      dtype=tf.float32,
                                                                      swap_memory=True, time_major=False)
            with tf.variable_scope("predictor", reuse=True):
                out_autoregressive, self.controller_final_state_autoregressive = tf.nn.dynamic_rnn(cell=cell_autoregressive,
                                                                                              inputs=rnn_inputs_autoregressive,
                                                                                              sequence_length=[configs.SEQ_LEN] * configs.BATCH_SIZE,
                                                                                              initial_state=self.controller_initial_state_autoregressive,
                                                                                              dtype=tf.float32,
                                                                                              swap_memory=True,
                                                                                              time_major=False)

            mse_gt = tf.reduce_mean(tf.squared_difference(out_gt, targets_normalized))
            mse_autoregressive = tf.reduce_mean(tf.squared_difference(out_autoregressive, targets_normalized))
            self.mse_autoregressive_steering = tf.reduce_mean(tf.squared_difference(out_autoregressive[:, :, 0], targets_normalized[:, :, 0]))
            steering_predictions = (out_autoregressive[:, :, 0] * self.std[0]) + self.mean[0]

            total_loss = self.mse_autoregressive_steering  # + 0.1 * (mse_gt + mse_autoregressive)

            optimizer = helper.get_optimizer(total_loss, learning_rate)

            tf.summary.scalar("MAIN TRAIN METRIC: rmse_autoregressive_steering", tf.sqrt(self.mse_autoregressive_steering))
            tf.summary.scalar("rmse_gt", tf.sqrt(mse_gt))
            tf.summary.scalar("rmse_autoregressive", tf.sqrt(mse_autoregressive))

            summaries = tf.summary.merge_all()
            train_writer = tf.summary.FileWriter('v3/train_summary', graph=graph)
            valid_writer = tf.summary.FileWriter('v3/valid_summary', graph=graph)
            saver = tf.train.Saver(write_version=tf.train.SaverDef.V2)

        return graph, saver, optimizer, steering_predictions, summaries, train_writer, valid_writer

    def __init__(self):

        # loading the dataset
        (self.train_seq, self.valid_seq), (self.mean, self.std) = utils.process_csv(filename=configs.TRAIN_DS_PATH, val=5)  # concatenated interpolated.csv from rosbags

        # interpolated.csv for testset filled with dummy values
        self.test_seq = utils.read_csv(configs.TEST_DS_PATH)

        # building the graph
        self.layer_norm = lambda x: tf.contrib.layers.layer_norm(inputs=x, center=True, scale=True, activation_fn=None, trainable=True)
        self.graph, self.saver, self.optimizer, self.steering_predictions, self.summaries, self.train_writer, self.valid_writer = self.build_graph()

        # built the graph but not train


