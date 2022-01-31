#!/usr/bin/env python2
from PIL import Image
import numpy as np
import os
import sys

def to_rgb(im, index):
    w, h = im.shape
    ret = np.empty((w, h, 3), dtype = np.uint8)
    ret[:, :, index] = im
    return ret

def to_color(img1, img2, img3):
    w, h, d = img1.shape
    ret = np.empty((w, h, 3), dtype = np.uint8)
    ret[:, :, 0] = img1[:, :, 0]
    ret[:, :, 1] = img2[:, :, 1]
    ret[:, :, 2] = img3[:, :, 2]
    return ret

def SSD(img1, img2, offset_Vector):
    OY = offset_Vector[1]
    OX = offset_Vector[0]
    l = 50

    # x, y, d = img1.shape
    # sub_img1 = img1[y//2 : y//2 + l, x//2: x//2 + l, :]
    # sub_img2 = img2[y//2 + OY : y//2 + l + OY, x//2 + OX : x//2 + l + OX, :]
    x, y = img1.shape
    sub_img1 = img1[y//2 : y//2 + l, x//2: x//2 + l]
    sub_img2 = img2[y//2 + OY : y//2 + l + OY, x//2 + OX : x//2 + l + OX]

    ssd = np.sum(np.square((sub_img1 - sub_img2))) #Take a sub set of the image and look for the similarity so you
                                                   #dont have to be getting unfavorable data in the shift
    return ssd

def NCC(img1, img2, offset_Vector):
    OY = offset_Vector[1]
    OX = offset_Vector[0]
    l = 50

    # x, y, d = img1.shape
    # sub_img1 = img1[y//2 : y//2 + l, x//2: x//2 + l, :]
    # sub_img2 = img2[y//2 + OY : y//2 + l + OY, x//2 + OX : x//2 + l + OX, :]
    x, y = img1.shape
    sub_img1 = img1[y//2 : y//2 + l, x//2: x//2 + l]
    sub_img2 = img2[y//2 + OY : y//2 + l + OY, x//2 + OX : x//2 + l + OX]

    product = np.mean((sub_img1 - sub_img1.mean()) * (sub_img2 - sub_img2.mean()))
    stds = sub_img1.std() * sub_img2.std()
    if stds == 0:
        return 0
    else:
        product /= stds
        return product

if __name__ == '__main__':
    workingDir = os.path.dirname(os.path.abspath(__file__))
    image = sys.argv[1]
    image_split = image.rsplit(".", 1)
    working_image_path = workingDir + '/' + image
    im = np.array(Image.open(working_image_path))

    rgb = np.array_split(im, 3)
    r = rgb[2]
    b = np.resize(rgb[0], r.shape)
    g = np.resize(rgb[1], r.shape)

    to_blue = to_rgb(b, 2)
    to_green = to_rgb(g, 1)
    to_red = to_rgb(r, 0)

    color = to_color(to_red, to_green, to_blue)
    color_img = Image.fromarray(color)
    color_img.save(image_split[0] + '-color.' + image_split[1])
    
    #Sum of Squard Distances
    mSSD_RB = (0, [0,0])
    mSSD_RG = (0, [0,0])
    for i in range(-15, 15):
        for j in range(-15, 15):
            offset_Vector = [j,i]
            # SSD_RB = SSD(to_red, to_blue, offset_Vector)
            # SSD_RG = SSD(to_red, to_green, offset_Vector)
            SSD_RB = SSD(r, b, offset_Vector)
            SSD_RG = SSD(r, g, offset_Vector)
            if mSSD_RB[0] < SSD_RB:
                mSSD_RB = (SSD_RB, offset_Vector)
            if mSSD_RG[0] < SSD_RG:
                mSSD_RG = (SSD_RG, offset_Vector)

    n = lambda s: s if s < 0 else None
    m = lambda s: max(0,s)
    x = mSSD_RB[1][1]
    y = mSSD_RB[1][0]
    to_blue_shifted = np.zeros_like(to_blue)
    to_blue_shifted[m(y):n(y), m(x):n(x)] = to_blue[m(-y):n(-y), m(-x):n(-x)]
    x = mSSD_RG[1][1]
    y = mSSD_RG[1][0]
    to_green_shifted = np.zeros_like(to_green)
    to_green_shifted[m(y):n(y), m(x):n(x)] = to_green[m(-y):n(-y), m(-x):n(-x)]
    # to_blue_shifted = np.roll(to_blue, mSSD_RB[1])
    # to_green_shifted = np.roll(to_green, mSSD_RG[1])

    ssd_color = to_color(to_red, to_green_shifted, to_blue_shifted)
    ssd_color_img = Image.fromarray(ssd_color)
    ssd_color_img.save(image_split[0] + '-ssd.' + image_split[1])

    print(str(image_split[0]) + '-ssd.' + str(image_split[1]) + " SSD Alignment")
    print("Blue = " + str(mSSD_RB[1]))
    print("Green = " + str(mSSD_RG[1]))

    #Normalized Cross Correlation
    mNCC_RB = (0, [0,0])
    mNCC_RG = (0, [0,0])
    for i in range(-15, 15):
        for j in range(-15, 15):
            offset_Vector = [j,i]
            # NCC_RB = NCC(to_red, to_blue, offset_Vector)
            # NCC_RG = NCC(to_red, to_green, offset_Vector)
            NCC_RB = NCC(r, b, offset_Vector)
            NCC_RG = NCC(r, g, offset_Vector)
            if mNCC_RB[0] < NCC_RB:
                mNCC_RB = (NCC_RB, offset_Vector)
            if mNCC_RG[0] < NCC_RG:
                mNCC_RG = (NCC_RG, offset_Vector)

    n = lambda s: s if s < 0 else None
    m = lambda s: max(0,s)
    x = mNCC_RB[1][1]
    y = mNCC_RB[1][0]
    to_blue_shifted = np.zeros_like(to_blue)
    to_blue_shifted[m(y):n(y), m(x):n(x)] = to_blue[m(-y):n(-y), m(-x):n(-x)]
    x = mNCC_RG[1][1]
    y = mNCC_RG[1][0]
    to_green_shifted = np.zeros_like(to_green)
    to_green_shifted[m(y):n(y), m(x):n(x)] = to_green[m(-y):n(-y), m(-x):n(-x)]
    # to_blue_shifted = np.roll(to_blue, mNCC_RB[1])
    # to_green_shifted = np.roll(to_green, mNCC_RG[1])

    ncc_color = to_color(to_red, to_green_shifted, to_blue_shifted)
    ncc_color_img = Image.fromarray(ncc_color)
    ncc_color_img.save(image_split[0] + '-ncc.' + image_split[1])

    print(str(image_split[0]) + '-ncc.' + str(image_split[1]) + " NCC Alignment")
    print("Blue = " + str(mNCC_RB[1]))
    print("Green = " + str(mNCC_RG[1]))