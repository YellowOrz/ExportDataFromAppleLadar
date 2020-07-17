import cv2 as cv
import os
import numpy as np
dir_path = "./Documents/"
R=np.array([255,0,0], dtype=np.uint8)
G=np.array([0,255,0], dtype=np.uint8)
B=np.array([0,0,255], dtype=np.uint8)
confidence2RGB={0:R,1:G,2:B} # 0、1、2分别对应置信度低、中、高
for filename in os.listdir(dir_path):
    if "txt" in filename:
        i = 0; x = 0; y = 0
        f = open(dir_path+filename)
        
        # 深度图可视化
        if "depth" in filename:
            img_np = np.zeros((192, 256), dtype=float)
            value = f.readline()
            while value:
                # print("({}, {}) = {}".format(x,y,value))
                img_np[y][x] = value
                i=i+1
                x = x+1
                if (i % 256) == 0:
                    x=0
                    y=y+1
                value = f.readline()
            # print(i)
            max=img_np.max()
            min=img_np.min()
            # 限制最大深度为3米
            if max > 3000:
                max = 3000
            # print("max = {}, min = {}".format(max,min))
            # 将深度信息归一化到0-255
            img_np=(img_np-min)*255/(max-min)
            img=cv.merge([img_np])
            img_name = (dir_path+filename+".png").replace(".txt","")
            cv.imwrite(img_name, img)
            print("succeed: {}".format(filename))
        
        # 置信图可视化
        elif "confidence" in filename:
            img=np.zeros((192, 256,3), dtype=np.uint8)
            value = f.readline()
            while value:
                confidence = int(value)
                # print("({}, {}) = {}".format(x,y,value))
                if (confidence is 0) or (confidence is 1) or (confidence is 2):
                    img[y][x] = confidence2RGB[confidence]
                else:
                    print("### Error: confidenceMap has wrong value -- {}".format(value))
                    exit
                i=i+1
                x = x+1
                if (i % 256) == 0:
                    x=0
                    y=y+1
                value = f.readline()
            img = cv.cvtColor(img, cv.COLOR_BGR2RGB) 
            img_name = (dir_path+filename+".png").replace(".txt","")
            cv.imwrite(img_name, img)
            print("succeed: {}".format(filename))

        else:
            print("failed:      {}".format(filename))
        
    
