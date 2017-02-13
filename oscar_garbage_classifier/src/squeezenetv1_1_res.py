import h5py
from keras.models import Model
from keras.layers import Input, Convolution2D, MaxPooling2D, Dropout, GlobalAveragePooling2D, merge, Activation, ZeroPadding2D
from keras.layers import AveragePooling2D, Flatten, Dense
from keras.layers.advanced_activations import LeakyReLU
from keras.layers.normalization import BatchNormalization
from keras import backend as K


def FireModule(s_1x1, e_1x1, e_3x3, name):
    """FireModule

        Fire module for the SqueezeNet model. 
        Implements the expand layer, which has a mix of 1x1 and 3x3 filters, 
        by using two conv layers concatenated in the channel dimension. 

    :param s_1x1: Number of 1x1 filters in the squeeze layer
    :param e_1x1: Number of 1x1 filters in the expand layer
    :param e_3x3: Number of 3x3 filters in the expand layer
    :param name: Name of the fire module
    :return: 
        Returns a callable function
    """
    # Concat on the channel axis. TensorFlow uses (rows, cols, channels), while
    # Theano uses (channels, rows, cols).
    if K.image_dim_ordering() == 'tf':
        concat_axis = 3
    else:
        concat_axis = 1

    def layer(x):
        squeeze = Convolution2D(s_1x1, 1, 1, border_mode='valid', init='glorot_uniform', name=name+'/squeeze1x1')(x)
        squeeze = BatchNormalization(name=name+'/squeeze1x1_bn')(squeeze)
        squeeze = LeakyReLU()(squeeze)

        # Needed to merge layers expand_1x1 and expand_3x3.
        expand_1x1 = Convolution2D(e_1x1, 1, 1, border_mode='valid', init='glorot_uniform', name=name+'/expand1x1')(squeeze)
        expand_1x1 = BatchNormalization(name=name+'/expand1x1_bn')(expand_1x1)
        expand_1x1 = LeakyReLU()(expand_1x1)

        # Pad the border with zeros. Not needed as border_mode='valid' will do the valid.
        # expand_3x3 = ZeroPadding2D(padding=(1, 1), name=name+'_expand_3x3_padded')(squeeze)
        expand_3x3 = Convolution2D(e_3x3, 3, 3, border_mode='same', init='glorot_uniform', name=name+'/expand3x3')(squeeze)
        expand_3x3 = BatchNormalization(name=name+'/expand3x3_bn')(expand_3x3)
        expand_3x3 = LeakyReLU()(expand_3x3)
        # Concat in the channel dim
        expand_merge = merge([expand_1x1, expand_3x3], mode='concat', concat_axis=concat_axis, name=name+'/concat')
        return expand_merge
    return layer
    

def SqueezeNet(nb_classes, rows=227, cols=227, channels=3):
    """
        SqueezeNet v1.1 implementation
    :param nb_classes: Number of classes
    :param rows: Amount of rows in the input
    :param cols: Amount of cols in the input
    :param channels: Amount of channels in the input
    :returns: SqueezeNet model
    """
    print("SqueezeNet V1.1")
    if K.image_dim_ordering() == 'tf':
        input_shape = (rows, cols, channels)
    else:
        input_shape = (channels, rows, cols)

    input_image = Input(shape=input_shape)
    conv1 = Convolution2D(64, 3, 3, subsample=(2, 2), border_mode='valid', init='glorot_uniform', name='conv1')(input_image)
    conv1 = BatchNormalization(name='conv1_bn')(conv1)
    conv1 = LeakyReLU()(conv1)

    maxpool1 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), border_mode='valid', name='pool1')(conv1)
    fire2 = FireModule(s_1x1=16, e_1x1=64, e_3x3=64, name='fire2')(maxpool1)
    fire3 = FireModule(s_1x1=16, e_1x1=64, e_3x3=64, name='fire3')(fire2)
    res3 = merge([fire2, fire3], mode='sum')
    maxpool3 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), border_mode='valid', name='pool3')(res3)
    fire4 = FireModule(s_1x1=32, e_1x1=128, e_3x3=128, name='fire4')(maxpool3)
    fire5 = FireModule(s_1x1=32, e_1x1=128, e_3x3=128, name='fire5')(fire4)
    res5 = merge([fire4, fire5], mode='sum')
    maxpool5 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), border_mode='valid', name='pool5')(res5)
    fire6 = FireModule(s_1x1=48, e_1x1=192, e_3x3=192, name='fire6')(maxpool5)
    fire7 = FireModule(s_1x1=48, e_1x1=192, e_3x3=192, name='fire7')(fire6)
    res7 = merge([fire6, fire7], mode='sum')
    fire8 = FireModule(s_1x1=64, e_1x1=256, e_3x3=256, name='fire8')(res7)
    fire9 = FireModule(s_1x1=64, e_1x1=256, e_3x3=256, name='fire9')(fire8)
    res9 = merge([fire8, fire9], mode='sum')
    # Dropout after fire9 module.
    fire9 = Dropout(p=0.5, name='dropout9')(res9)
    conv10 = Convolution2D(nb_classes, 1, 1, init='he_normal', border_mode='valid', name='conv10')(fire9)
    conv10 = LeakyReLU()(conv10)
    avgpool10 = GlobalAveragePooling2D(name='pool10')(conv10)
    loss = Activation('softmax', name='softmax')(avgpool10)
    # loss = Activation('sigmoid', name='sigmoid')(avgpool10)

    model = Model(input=input_image, output=loss)
    return model



