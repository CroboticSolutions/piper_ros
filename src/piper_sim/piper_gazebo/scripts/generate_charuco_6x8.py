#!/usr/bin/env python3
"""Exact metric ChArUco mesh for Gazebo; no texture filtering or external assets."""
from pathlib import Path
import struct
import cv2
import numpy as np


def main():
    destination=Path(__file__).resolve().parents[1]/'models'/'charuco_6x8_25mm'
    destination.mkdir(parents=True,exist_ok=True)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board=cv2.aruco.CharucoBoard((6,8),.025,.018,dictionary)
    # 100 px/square, 72 px/marker, 12 px/marker-bit including its black border.
    pattern=board.generateImage((600,800),marginSize=0,borderBits=1)
    cv2.imwrite(str(destination/'reference.png'),cv2.copyMakeBorder(pattern,40,40,40,40,cv2.BORDER_CONSTANT,value=255))
    triangles=[];scale=.025/100
    for row in range(pattern.shape[0]):
        black=np.r_[False,pattern[row]<128,False].astype(np.int8)
        start=np.flatnonzero(np.diff(black)==1);end=np.flatnonzero(np.diff(black)==-1)
        for a,b in zip(start,end):
            x0,x1=(a-300)*scale,(b-300)*scale
            y0,y1=(400-row-1)*scale,(400-row)*scale
            triangles.extend([((x0,y0,0),(x1,y0,0),(x1,y1,0)),((x0,y0,0),(x1,y1,0),(x0,y1,0))])
    with (destination/'pattern.stl').open('wb') as f:
        f.write(b'ChArUco 6x8 square25mm marker18mm DICT_4X4_100'.ljust(80,b' '));f.write(struct.pack('<I',len(triangles)))
        for tri in triangles:
            f.write(struct.pack('<12fH',0,0,1,*np.asarray(tri).ravel(),0))
    mesh=(destination/'pattern.stl').as_uri()
    (destination/'model.sdf').write_text(f'''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="charuco_board_6x8">
    <static>true</static>
    <pose>0.50 -0.25 0 0 0 0</pose>
    <link name="board">
      <collision name="plate"><pose>0 0 0.001 0 0 0</pose><geometry><box><size>0.170 0.220 0.002</size></box></geometry></collision>
      <visual name="white_backing"><pose>0 0 0.001 0 0 0</pose><geometry><box><size>0.170 0.220 0.002</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular></material></visual>
      <visual name="charuco_pattern"><pose>0 0 0.00202 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><mesh><uri>{mesh}</uri></mesh></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular></material></visual>
    </link>
  </model>
</sdf>
''')
    (destination/'model.config').write_text('<model><name>ChArUco 6x8 25mm 18mm</name><version>1.0</version><sdf version="1.8">model.sdf</sdf><description>DICT_4X4_100; pattern150x200mm; white backing170x220mm; thickness2mm.</description></model>\n')
    corners,ids,markers,marker_ids=cv2.aruco.CharucoDetector(board).detectBoard(cv2.imread(str(destination/'reference.png'),cv2.IMREAD_GRAYSCALE))
    assert ids is not None and len(ids)==35
    assert marker_ids is not None and len(marker_ids)==24
    print(f'{destination}: {len(ids)} ChArUco corners, {len(marker_ids)} markers, {len(triangles)} mesh triangles')


if __name__=='__main__':main()
