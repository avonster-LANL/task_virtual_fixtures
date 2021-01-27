#!/usr/bin/env python

import numpy as np
from scipy import misc
import matplotlib.pyplot as plt


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])


def main():
    path = '/home/andrewsharp/catkin_ws/src/task_virtual_fixtures/tvf_data/image_data/'
    images = ['U1S1',
              'U1S2',
              'U1S3',
              'U2S1',
              'U2S2',
              'U2S3',
              'U3S1',
              'U3S2',
              'U4S1',
              'U4S2',
              'U4S3']
    file_extension = '.jpg'

    for i in images:
        print(i)
        image = misc.imread(path + i + '_resized' + file_extension)
        image_analyzed = misc.imread(path + i + '_resized' + file_extension)
        # image_shape = image.shape
        # image_mean = image.mean()
        # print(image_shape)
        # print(image_mean)

        # image = image[650: 4950, 650: 8050]
        # misc.imsave(path + i + file_suffix + file_extension, image)

        # image = rgb2gray(image)
        # image_shape = image.shape
        # image_mean = image.mean()
        # print(image_shape)
        # print(image_mean)

        # image_shape = image.shape
        # image_mean = image.mean()
        # print(image_shape)
        # print(image_mean)

        # plt.imshow(image)
        # plt.show()
        print("Stats")
        print(image.mean(), np.std(image))
        '''
        overclean = 0.0
        clean = 0.0
        underclean = 0.0
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                if int(image[x][y][1]) < 100:
                    overclean += 1
                    image_analyzed[x][y] = [0, 0, 0]
                elif 50 <= int(image[x][y][0]) - int(image[x][y][2]):
                    clean += 1
                    image_analyzed[x][y] = [125, 125, 125]
                elif int(image[x][y][0]) - int(image[x][y][2]) < 50:
                    underclean += 1
                    image_analyzed[x][y] = [255, 255, 255]

        print("Over clean: ")
        print(overclean / (image.shape[0] * image.shape[1]) * 100)
        print("Clean: ")
        print(clean / (image.shape[0] * image.shape[1]) * 100)
        print("Under clean: ")
        print(underclean / (image.shape[0] * image.shape[1]) * 100)

        misc.imsave(path + i + "_analyzed" + file_extension, image_analyzed)
        # plt.imshow(image)
        # plt.show()
        # plt.imshow(image_analyzed)
        # plt.show()
        '''

if __name__ == '__main__':
    main()


