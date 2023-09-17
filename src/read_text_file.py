#!/usr/bin/env python3

import quaternion
import numpy as np
import csv

quaternions=[]
filename='src/IMU_ADQ_pck/src/others/quats.txt'
with open('src/IMU_ADQ_pck/src/participant_record/Part 1_High Stiff Part1.txt',newline = '') as f:
    spamreader = csv.reader(f, delimiter=',', quotechar='|')
    for row in spamreader:
        q=quaternion.from_euler_angles(*row[:3])
        with open(filename, 'a') as file:
                    file.write(f'{q.w} , {q.x} , {q.y} , {q.z} \n')

print("done")        

