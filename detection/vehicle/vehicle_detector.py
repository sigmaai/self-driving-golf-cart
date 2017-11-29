
def detect_vechicle(image):

     
    # -----------------------------------------------------
    # Check to see if the user closed the window
    if cv2.getWindowProperty(windowName, 0) < 0:
        # This will fail if the user closed the window; get printed to the console
        break
    ret_val, image = cap.read()

    # -----------------------------------------------------
    # run the network and detection

    resized_image = cv2.resize(image, (416, 416))
    image_data = np.array(resized_image, dtype='float32')

    image_data /= 255.
    image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

    # making predictions
    out_boxes, out_scores, out_classes = sess.run(
        [boxes, scores, classes],
        feed_dict={
            yolo_model.input: image_data,
            input_image_shape: [image.shape[1], image.shape[0]],
            K.learning_phase(): 0
        })

    font = ImageFont.truetype(font='font/FiraMono-Medium.otf',
                              size=np.floor(3e-2 * image.shape[1] + 0.5).astype('int32'))
    thickness = (image.shape[0] + image.shape[1]) // 300

    # draw the bounding boxes
    for i, c in reversed(list(enumerate(out_classes))):
        predicted_class = class_names[c]
        box = out_boxes[i]
        score = out_scores[i]

        label = '{} {:.2f}'.format(predicted_class, score)
        draw = ImageDraw.Draw(image)
        label_size = draw.textsize(label, font)

        top, left, bottom, right = box
        top = max(0, np.floor(top + 0.5).astype('int32'))
        left = max(0, np.floor(left + 0.5).astype('int32'))
        bottom = min(image.shape[1], np.floor(bottom + 0.5).astype('int32'))
        right = min(image.shape[0], np.floor(right + 0.5).astype('int32'))

        if top - label_size[1] >= 0:
            text_origin = np.array([left, top - label_size[1]])
        else:
            text_origin = np.array([left, top + 1])

        # My kingdom for a good redistributable image drawing library.
        for i in range(thickness):
            draw.rectangle([left + i, top + i, right - i, bottom - i], outline=colors[c])
        draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=colors[c])
        draw.text(text_origin, label, fill=(0, 0, 0), font=font)
        del draw

    displayBuf = image

    # show the stuff
    # -----------------------------------------------------

    cv2.imshow(windowName, displayBuf)
    key = cv2.waitKey(10)
    if key == 27:  # ESC key
        cv2.destroyAllWindows()
        break