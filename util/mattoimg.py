#!/usr/bin/python3

import cv2
import sys

def saveImg(infilename, outfilename = ''):
    if outfilename == '':
        outfilename = infilename.rsplit('.', 1)[0] + '.bmp'
    cvfile = cv2.FileStorage(infilename, cv2.FILE_STORAGE_READ)
    mat = cvfile.getFirstTopLevelNode().mat()
    cvfile.release()
    cv2.imwrite(outfilename, mat)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        saveImg(sys.argv[1])
    elif len(sys.argv) == 3:
        saveImg(sys.argv[1], sys.argv[2])
    else:
        print('Usage: ', sys.argv[0], 'filename.xml [filename.bmp]')
