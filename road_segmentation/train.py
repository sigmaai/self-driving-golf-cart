import model
import configs
import utils
from keras.optimizers import Adam

df_vehicles = utils.prepare_database(configs.data_path)

smooth = 1.
model = model.fcn_model()
model.compile(optimizer=Adam(lr=1e-4), loss=model.IOU_calc_loss, metrics=[model.IOU_calc])

training_gen = utils.generate_train_batch(df_vehicles, 32)
model.fit_generator(training_gen, steps_per_epoch=500, epochs=1, verbose=1, callbacks=None, validation_data=None)
model.save('./train-1.h5')
