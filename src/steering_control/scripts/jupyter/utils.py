#
# Utility File
# Self-driving vehicle research
# Sigma A.I.
# (c) Yongyang Nie, 2018
# Contact: contact@neilnie.com
#


import numpy as np
import tensorflow as tf

SEQ_LEN = 10 # this parameter can be changed. TODO try longer sequences if memory is available.
BATCH_SIZE = 4 # this parameter can also be changed
LEFT_CONTEXT = 5

HEIGHT = 480
WIDTH = 640
CHANNELS = 3

RNN_SIZE = 32
RNN_PROJ = 32

CSV_HEADER = "index,timestamp,width,height,frame_id,filename,angle,torque,speed,lat,long,alt".split(",")
OUTPUTS = CSV_HEADER[-6:-3] # angle,torque,speed
OUTPUT_DIM = len(OUTPUTS) # predict all features


class BatchGenerator(object):
    def __init__(self, sequence, seq_len, batch_size):
        self.sequence = sequence
        self.seq_len = seq_len
        self.batch_size = batch_size
        chunk_size = 1 + (len(sequence) - 1) / batch_size
        self.indices = [(i*chunk_size) % len(sequence) for i in range(batch_size)]
        
    def next(self):
        while True:
            output = []
            for i in range(self.batch_size):
                idx = int(self.indices[i])
                left_pad = self.sequence[idx - LEFT_CONTEXT:idx]
                if len(left_pad) < LEFT_CONTEXT:
                    left_pad = [self.sequence[0]] * (LEFT_CONTEXT - len(left_pad)) + left_pad
                assert len(left_pad) == LEFT_CONTEXT
                leftover = len(self.sequence) - idx
                if leftover >= self.seq_len:
                    result = self.sequence[idx:idx + self.seq_len]
                else:
                    result = self.sequence[idx:] + self.sequence[:self.seq_len - leftover]
                assert len(result) == self.seq_len
                self.indices[i] = (idx + self.seq_len) % len(self.sequence)
                images, targets = zip(*result)
                images_left_pad, _ = zip(*left_pad)
                output.append((np.stack(images_left_pad + images), np.stack(targets)))
            output = list(zip(*output))
            output = [np.stack(output[0]), np.stack(output[1])]
            return output
        
        
def read_csv(filename):
    with open(filename, 'r') as f:
        lines = [ln.strip().split(",")[-7:-3] for ln in f.readlines()]
        lines = map(lambda x: (x[0], np.float32(x[1:])), lines) # imagefile, outputs
        return lines

    
def process_csv(filename, val=5):
    sum_f = np.float128([0.0] * OUTPUT_DIM)
    sum_sq_f = np.float128([0.0] * OUTPUT_DIM)
    lines = read_csv(filename)
    # leave val% for validation
    train_seq = []
    valid_seq = []
    cnt = 0
    for ln in lines:
        if cnt < SEQ_LEN * BATCH_SIZE * (100 - val): 
            train_seq.append(ln)
            sum_f += ln[1]
            sum_sq_f += ln[1] * ln[1]
        else:
            valid_seq.append(ln)
        cnt += 1
        cnt %= SEQ_LEN * BATCH_SIZE * 100
    mean = sum_f / len(train_seq)
    var = sum_sq_f / len(train_seq) - mean * mean
    std = np.sqrt(var)
    print("train sequence length: " + str(len(train_seq)) + "\n" \
          + "val sequence length: " + str(len(valid_seq)))
    print("means: " + str(mean) + "\n" \
          + "standard deviation: " + str(std))
    return (train_seq, valid_seq), (mean, std)


def get_initial_state(complex_state_tuple_sizes):
    flat_sizes = tf.contrib.framework.nest.flatten(complex_state_tuple_sizes)
    init_state_flat = [tf.tile(
        multiples=[BATCH_SIZE, 1], 
        input=tf.get_variable("controller_initial_state_%d" % i, initializer=tf.zeros_initializer, shape=([1, s]), dtype=tf.float32))
        for i,s in enumerate(flat_sizes)]
    init_state = tf.contrib.framework.nest.pack_sequence_as(complex_state_tuple_sizes, init_state_flat)
    return init_state


def deep_copy_initial_state(complex_state_tuple):
    flat_state = tf.contrib.framework.nest.flatten(complex_state_tuple)
    flat_copy = [tf.identity(s) for s in flat_state]
    deep_copy = tf.contrib.framework.nest.pack_sequence_as(complex_state_tuple, flat_copy)
    return deep_copy


def get_optimizer(loss, lrate):
        optimizer = tf.train.AdamOptimizer(learning_rate=lrate)
        gradvars = optimizer.compute_gradients(loss)
        gradients, v = zip(*gradvars)
        print([x.name for x in v])
        gradients, _ = tf.clip_by_global_norm(gradients, 15.0)
        return optimizer.apply_gradients(zip(gradients, v))
    
