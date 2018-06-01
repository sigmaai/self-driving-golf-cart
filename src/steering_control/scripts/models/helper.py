
import tensorflow as tf
import configs

# Our training data follows the "interpolated.csv" format from Ross Wightman's scripts.


def get_optimizer(loss, lrate):

    optimizer = tf.train.AdamOptimizer(learning_rate=lrate)
    gradvars = optimizer.compute_gradients(loss)
    gradients, v = zip(*gradvars)
    print([x.name for x in v])
    gradients, _ = tf.clip_by_global_norm(gradients, 15.0)
    return optimizer.apply_gradients(zip(gradients, v))


def get_initial_state(complex_state_tuple_sizes):

    flat_sizes = tf.contrib.framework.nest.flatten(complex_state_tuple_sizes)
    init_state_flat = [tf.tile(multiples=[configs.BATCH_SIZE, 1],
                               input=tf.get_variable("controller_initial_state_%d" % i,
                                                     initializer=tf.zeros_initializer, shape=([1, s]),
                                                     dtype=tf.float32))
                       for i, s in enumerate(flat_sizes)
                       ]
    init_state = tf.contrib.framework.nest.pack_sequence_as(complex_state_tuple_sizes, init_state_flat)
    return init_state


def deep_copy_initial_state(complex_state_tuple):

    flat_state = tf.contrib.framework.nest.flatten(complex_state_tuple)
    flat_copy = [tf.identity(s) for s in flat_state]
    deep_copy = tf.contrib.framework.nest.pack_sequence_as(complex_state_tuple, flat_copy)
    return deep_copy
