<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [项目说明](#项目说明)
- [文件说明](#文件说明)
- [使用方法](#使用方法)
- [导出数据说明](#导出数据说明)
- [相机内参](#相机内参)

<!-- /code_chunk_output -->

# 项目说明
本项目基于Apple的官方源码[Visualizing a Point Cloud Using Scene Depth](https://developer.apple.com/documentation/arkit/visualizing_a_point_cloud_using_scene_depth?changes=latest_minor)，将ipad pro的dToF的相关数据（彩色图、深度图、置信图）保存到本地。  

# 文件说明
- 文件夹`VisualizingAPointCloudUsingSceneDepth`：基于Apple的官方源码[Visualizing a Point Cloud Using Scene Depth](https://developer.apple.com/documentation/arkit/visualizing_a_point_cloud_using_scene_depth?changes=latest_minor)进行修改，实现导出dToF数据的功能。项目中修改的代码文件如下：  
  - 增加`CVPixelBufferExtension.swift`：主要是对类`CVPixelBuffer`的功能扩展  
  - 修改`Renderer.swift`：增加了使用扩展功能来导出数据的代码  

  其余文件中，修改的地方应该只有增加了注释，并且所有**增加的注释**中都会出现**中文**
- 文档`RemoveString.sh`：去除ipad保存的深度、置信数据中的字符串（如下图）  
<img width=400 src=./Img_for_README/remove_string.png>

- 文档`txt2png.py`：可视化ipad保存的深度、置信数据  
- 文件夹`Img_for_README`：保存`README.md`中用到的图片  

# 使用方法

- 在MacOS上，使用xcode（**12.0以上版本**），“Open a Project or File”，选择文件夹`VisualizingAPointCloudUsingSceneDepth`，例如  
<img width=600 src=./Img_for_README/open_project.png>

- 将ipad pro连接到Mac电脑，选择设备为自己ipad，例如  
<img width=400 src=./Img_for_README/chose_device.png>

- 保持ipad为解锁状态，运行项目，然后ipad上会运行程序  
<img width=200 src=./Img_for_README/run.png>

- 采集完数据后，使用快捷键`command`+`shift`+`2`，或者xcode菜单栏打开`Window`->`Devices and Simulators`，选择ipad pro，然后将应用的**数据导出**，导出路径自定
<img width=600 src=./Img_for_README/export_data.png>

- 导出数据后，后xcode会自动打开访达。对着导出的文件右键，选择`显示包内容`。然后进入文件夹`AppData`，里面的文件夹`Documents`下的文件就是ipad pro的iToF的相关数据，即**彩色图（PNG）、深度信息（无后缀）、置信信息（无后缀）**  
- 将整个`Documents`文件夹，放到本项目的根目录下面，然后**在本项目根目录下**运行`RemoveString.sh`，例如：  
    ```bash
    bash RemoveString.sh
    ```
    脚步处理完的文件仍然位于`Documents`，文件名增加了后缀`.txt`，然后**原始文件**被存放在了`Documents/RawData`
- （**可选**）如果想将深度信息、置信信息**可视化**，**在本项目根目录下**运行`txt2png.py`，例如：    
    ```bash
    # 需要numpy、opencv-python 
    python txt2png.py
    ```

# 导出数据说明
- 原始数据：从ipad pro导出的数据。不同场景的数据通过文件名中的时间（格式为“月-日-小时-分钟-秒-毫秒”）以区分，例如  
<img width=600 src=./Img_for_README/raw_data.png>


  - `07-17-19-13-48-038_RGB.png`： 当前场景的彩色图，大小为3840*2880像素 
  - `07-17-19-13-48-038_depthMap_Matrix`：当前场景的深度信息，通过**行遍历**原始深度图信息获得，文件中一行对应一个像素点，单位为mm，总共49152个点（对应深度图的大小256*192）  
  - `07-17-19-13-48-038_confidenceMap_Matrix`：当前场景的置信信息，通过**行遍历**原始置信图信息获得，文件中一行对应一个像素点，取值为0、1、2（对应置信度为低、中、高），总共49152个点（对应置信图的大小256*192）  
- 处理后的数据：通过脚本`RemoveString.sh`和`RemoveString.sh`处理后的数据。  
<img width=300 src=./Img_for_README/proceed_data1.png>  
<img width=600 src=./Img_for_README/proceed_data2.png>  

  - `RawData`：存放了深度信息、置信信息的原始文件  
  - `07-17-19-13-48-038_RGB.png`：无任何处理，跟上面的一样  
  - `07-17-19-13-48-038_depthMap_Matrix.txt`：去除了深度信息的原始文件中的字符串，只保留数字，原始文件被存放在文件夹`RawData`下面  
  - `07-17-19-13-48-038_depthMap_Matrix.png`：将深度信息可视化（灰度图）
  > 注意：`txt2png.py`中将深度值的上限设为3米
  - `07-17-19-13-48-038_confidenceMap_Matrix.txt`：去除了置信信息的原始文件中的字符串，只保留数字，原始文件被存放在文件夹`RawData`下面  
  - `07-17-19-13-48-038_confidenceMap_Matrix.png`：将置信信息可视化，颜色**红、绿、蓝**分别对应置信度**低、中、高**

# 相机内参
相机内参获取方法如下：
- 在`VisualizingAPointCloudUsingSceneDepth/Renderer.swift`中的语句`let cameraIntrinsicsInversed = camera.intrinsics.inverse`处加断点  
- 运行程序，在断点处停下后，在lldb窗口中输入如下命令即可获取  
  ```
  po camera.intrinsics
  ```

<img width=600 src=./Img_for_README/get_intrinsiccs.png>  

> lldb窗口打开方式：xcode菜单栏`View`->`Debug Area`->`Activate Console`，或者快捷键`command`+`shift`+`c`