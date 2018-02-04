# coding=utf-8
from keras.utils import plot_model


def select_model(model_name):
    if model_name == 'enet_unpooling':
        from .enet_unpooling import model
    elif model_name == 'enet_naive_upsampling':
        from .enet_naive_upsampling import model
    elif model_name == 'icnet':
        from .icnet import model
    else:
        raise ValueError('Unknown model {}'.format(model_name))
    return model


def plot(model_name):
    model = select_model(model_name=model_name)
    autoencoder, name = model.build(nc=2, w=512, h=512)
    plot_model(autoencoder, to_file='{}.png'.format(name), show_shapes=True)
