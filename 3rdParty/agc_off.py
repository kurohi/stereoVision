#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable-msg=C0103
"""
tool for AGC AWB off tool for Sikino hitech camera.
http://www.shikino.co.jp/products/product-kbcr-s05vu.html

sikino:
    register_rw_function.c
    gcc register_rw_function.c -o sikino

    source code location:
    (SVN)/computer_vision/hardware/camera

    Note:
    The source code is for Linux
    The source code includes
    #include <linux/videodev2.h>


See also:
hex 00 means all mode are manual.

OV7725_CSP2_DS (1 4).pdf
Table 6 device Control register List(Sheet 4 of 14) in page 15.
    OV7725 Color CMOS VGA (640x480) CAMERACHIP Sensor

This python script was orignally written by waragai
"""

import os

def sikino_write_register(video, reg_adr, reg_val):
    """write to register in the video
    video: device file
    reg_adr: hex value
    reg_val: hex value
    """

    txtData = """w %x %x
q

""" % (reg_adr, reg_val)
    open("agc_off.txt", "wt").write(txtData)

    cmd = "./sikino %s < agc_off.txt" % video
    os.system(cmd)

def sikino_read_register(video, reg_adr):
    """read register in the video
    video: device file
    reg_adr: hex value
    """

    txtDataRead = """r %x
q

""" % reg_adr

    open("agc_read.txt", "wt").write(txtDataRead)

    cmd = "./sikino %s < agc_read.txt" % video
    os.system(cmd)

def AGC_AWB_off():
    """let AGC and AWB off
    """

    for video in ("/dev/video0", "/dev/video1"):
        if os.path.exists(video):
            sikino_write_register(video, 0x13, 0x00)
        else:
            print video + " is not found."

    for video in ("/dev/video0", "/dev/video1"):
        if os.path.exists(video):
            sikino_read_register(video, 0x13)
        else:
            print video + " is not found."

    print "done AGC_AWB_off()"


if __name__ == "__main__":
    AGC_AWB_off()
