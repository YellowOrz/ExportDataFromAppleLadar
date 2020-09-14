#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <Eigen/Core>
#include <pcl/console/parse.h>
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
using namespace std;
using namespace Eigen;

void GetDepthFiles(const string& dir, vector<string> &depth_files);
void GetCameraFiles(const string& dir, vector<string> &camera_files);
bool ReadIntrinsic(Matrix3d& intrinsic, const string &camera_path) ;
bool CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const Matrix3d& intrinsic, const string & depth_file);
int main(int argc, char **argv) {
    /* 解析参数，获取数据所在目录 */
    string dir;
    if (argc == 2) dir = argv[1];    // 注意：参数中的路径须以“/”结尾，不是的话程序会自动加上一个
    else {
        cout << RED << "Warning: " << RESET << "There is no entering of path of group 4 data!!!" << endl;
        dir = "/home/orz/Documents/测试数据/4/";
    }
    if (dir[dir.length() - 1] != '/') {
        cout << RED << "Warning: " << RESET
             << "Direction should end by '/'! But don't worry, the program will add it for you :)" << endl;
        dir += '/';
    }

    /* 获取depth和camera文件名 */
    vector<string> depth_files, camera_files;
    GetDepthFiles(dir,depth_files);
    GetCameraFiles(dir, camera_files);
    if(depth_files.size() != camera_files.size()){
        cout << RED << "Warning: " << RESET << "The numbers of depth and camera are not equal!!!" << endl;
        cout << "depth_files.size() = " << depth_files.size() << "\tcamera_files.size() = " << camera_files.size() << endl;
    }
    /* 遍历每一帧 */
    for(int i=0;i<camera_files.size();i++){
        string depth_file = depth_files[i], camera_file = camera_files[i];

        /* 读取相机内参到一个矩阵里面 */
        Matrix3d intrinsic = Matrix3d::Zero();
        if(!ReadIntrinsic(intrinsic, camera_file)){
            cout << RED << "Error: " << RESET << "read intrinsic failed!" << endl;
        }

        /* 读取depth到一个点云里面*/
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->width = 256;
        cloud->height = 192;
        cloud->resize(256 * 192);
        if(!CreatePointCloud(*cloud, intrinsic, depth_file)){
            cout << RED << "Error: " << RESET << "create point cloud failed!" << endl;
        }

        /* 点云可视化 */
        //pcl::visualization::CloudViewer viewer("cloud");
        //viewer.showCloud(cloud);
        //while (!viewer.wasStopped()) {}

        /* 保存点云 */
        string path=depth_file.replace(depth_file.find("txt"),3,"pcd");
        //path=path.replace(path.find("txt"),3,"pcd"); // 将路径中的两个“txt”都替换成“pcd”
        cout <<YELLOW << "Saving: " << RESET <<  path << endl;
        pcl::io::savePCDFileASCII(path, *cloud);

    }
    return 0;
}

void GetDepthFiles(const string& dir, vector<string> &depth_files){
    string shell = "find " + dir + " -name *_depth.txt | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for depth does not exit!" << endl;
        return;
    }
    string depth_file;
    while(getline(fp, depth_file))
        depth_files.push_back(depth_file);
    fp.close();
}

void GetCameraFiles(const string& dir, vector<string> &camera_files){
    string shell = "find " + dir + " -name *_camera.txt | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for intrinsic does not exit!" << endl;
        return;
    }
    string camera_path;
    while(getline(fp, camera_path))
        camera_files.push_back(camera_path);
    fp.close();
}

bool ReadIntrinsic(Matrix3d& intrinsic, const string &camera_path) {
    /* 从文件中读取内参的数据到string */
    ifstream fp(camera_path);
    if (!fp) {
        cout << RED << "Error: " << "file " << camera_path << " does not exit!" << RESET << endl;
        return false;
    } else cout << YELLOW << "Reading: " << RESET << camera_path << endl;
    string data;
    while (getline(fp, data)) {
        if (data == "cameraIntrinsics") {
            getline(fp, data);   //先将字符串从文件中提取出来
            break;
        }
    }
    fp.close();

    /* 将string格式的数据转成Matirx */
    int index = data.find('['); // 删除字符串中的所有"[]"
    while (index != string::npos) {
        data.erase(index, 1);
        index = data.find('[');
    }
    index = data.find(']');
    while (index != string::npos) {
        data.erase(index, 1);
        index = data.find(']');
    }

    index = data.find(' ');     // 删除空格
    while (index != string::npos) {
        data.erase(index, 1);
        index = data.find(' ');
    }

    index = data.find('(');     // 删除“()”及其前后的字符串
    data.erase(data.begin(), data.begin() + index + 1);
    data.erase(data.end() - 1);

    //char *strs = new char[data.length() + 1]; // 根据分隔符“,”分割字符串。先转化为char×类型
    //strcpy(strs, data.c_str());
    //char d = ',';
    //char *p = strtok(strs, &d);
    //int i = 0;
    //while (p) {
    //    intrinsic(i % 3, i / 3) = stod(p);
    //    p = strtok(nullptr, &d);
    //    i++;
    //}


    string shell = "echo "+data+"|sed  's/,/ /g' > temp.txt";
    system(shell.c_str());
    fp.open("temp.txt");
    string s;
    int i = 0;
    while(fp>>data){
        //cout << i << "     " << data  << endl;
        intrinsic(i % 3, i / 3) = stod(data);
        i++;
    }

    return true;
}

bool CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const Matrix3d& intrinsic, const string & depth_file){
    ifstream fp(depth_file);
    if (!fp) {
        cout << RED << "Error: " << RESET << depth_file << " does not exit!" << endl;
        return false;
    } else cout << YELLOW << "Reading: " << RESET << depth_file << endl;
    string data;
    for (int i = 0; i < cloud.size(); i++) {
        getline(fp, data);
        double depth = stod(data)/1000;
        Vector3d pixel(i / 256 * 7.5,   // 7.5是彩色图和深度图边长的比例
                                       (192 - i % 256) * 7.5,  //如果显示出来的pcd与ipad采集的点云呈现镜像的话，为(192 - i % 256) * 7.5，但是这样的话转为xyz格式，到Geometric Wrap中显示又是镜像的了
                                      1); // 像素坐标系(u, v, 1)
        Vector3d image_plane = intrinsic.inverse() * pixel;   // 成像平面坐标系(x', y', 1)
        //cout << image_plane << endl << intrinsic.inverse() << endl;
        cloud.points[i].x = image_plane(0) * depth;
        cloud.points[i].y = image_plane(1) * depth;
        cloud.points[i].z = depth;
        cloud.points[i].r = 100;        //颜色自定义
        cloud.points[i].g = 100;
        cloud.points[i].b = 100;

    }
    return true;
}