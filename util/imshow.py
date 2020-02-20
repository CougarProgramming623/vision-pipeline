#!/usr/bin/python3

import cv2
import sys

def doShow(filename, nodename = 'myframe'):
    cvfile = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    mat = cvfile.getNode(nodename).mat()
    cvfile.release()
    cv2.imshow(filename, mat)
    cv2.waitKey()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        doShow(sys.argv[1])
    elif len(sys.argv) == 3:
        doShow(sys.argv[1], sys.argv[2])
    else:
        print('Usage: ', sys.argv[0], 'filename.xml [node_name]')
