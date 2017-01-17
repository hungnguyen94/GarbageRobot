#!/usr/bin/env python

import cv2


def resize_image(img_path):
    # Load image with 3 channel colors
    img = cv2.imread(img_path, flags=-1)
    height = img.shape[0]
    width = img.shape[1]
    offset = int(round(max(height, width) / 2.0))

    # Add borders to the images.
    padded_img = cv2.copyMakeBorder(img, offset, offset, offset, offset, cv2.BORDER_CONSTANT)
    padded_height = padded_img.shape[0]
    padded_width = padded_img.shape[1]
    center_x = int(round(padded_width / 2.0))
    center_y = int(round(padded_height / 2.0))
    # Crop the square containing the full image.
    cropped_img = padded_img[center_y - offset: center_y + offset, center_x - offset: center_x + offset]

    # Resize image to 227, 227
    resized_image = cv2.resize(cropped_img, (227, 227))
    return resized_image


if __name__ == '__main__':
    import os
    import errno

    def mkdir_p(path):
        try:
            os.makedirs(path)
        except OSError as exc:  # Python >2.5
            if exc.errno == errno.EEXIST and os.path.isdir(path):
                pass
            else:
                raise

    images_dir = '/mnt/data/Development/ros/catkin_ws/images/webcam_img/'
    output_dir = '/mnt/data/Development/ros/catkin_ws/images/resized'

    for subdir, dirs, files in os.walk(images_dir):
        for f in files:
            if f.endswith('.jpg') or f.endswith('.png'):
                img_path = os.path.join(subdir, f)
                class_name = os.path.basename(subdir)
                output_path = os.path.join(output_dir, class_name, f)
                mkdir_p(os.path.join(output_dir, class_name))

                print('Current file: %s' % os.path.join(class_name, f))
                resized_img = resize_image(img_path)
                cv2.imwrite(output_path, resized_img)

