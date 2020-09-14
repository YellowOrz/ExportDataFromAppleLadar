#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
using namespace std;
using namespace Eigen;
using namespace cv;
#define WIDTH 256
#define HEIGHT 192

void GetDepthFiles(const string &dir, vector<string> &depth_files);

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
    vector<string> depth_files;
    GetDepthFiles(dir, depth_files);
    if (depth_files.size() == 0) {
        cout << RED << "Error: " << RESET << dir << " has no depth files" << endl;
        exit(0);
    }

    /* 遍历每一帧 */
    for (int i = 0; i < depth_files.size(); i++) {
        /* 读取深度值 */
        string depth_file = depth_files[i];
        ifstream fp(depth_file);
        if (!fp) {
            cout << RED << "Error: " << RESET << depth_file << " does not exit!" << endl;
            return false;
        } else cout << YELLOW << "Reading: " << RESET << depth_file << endl;
        string data;
        float min_depth = 10000,max_depth = 0;
        vector<float> depth;
        while(getline(fp,data))
            depth.push_back(stof(data));
        if(depth.size() != (WIDTH * HEIGHT)){
            cout << RED << "Error: " << RESET << depth_file << " has wrong number of points " << depth.size() << endl;
            exit(0);
        }

        /* 归一化为深度图 */
        min_depth = *min_element(depth.begin(), depth.end());
        max_depth = *max_element(depth.begin(), depth.end());
        max_depth = max_depth>7500?7500:max_depth; // 限制最大距离为7.5m
        Mat depthmap = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        float temp = 255.0 / (max_depth - min_depth);
        for (int i = 0; i < WIDTH * HEIGHT; i++) {
            depthmap.at<uchar>(i/WIDTH,i%WIDTH) = depth[i] * temp;
        }

        /* 保存 */
        string path = depth_file.replace(depth_file.find("txt"),3,"png");
        cout << YELLOW <<"Saving: " << RESET <<path << endl;
        cv::imwrite(path,depthmap);
    }
    return 0;
}

void GetDepthFiles(const string &dir, vector<string> &depth_files) {
    string shell = "find " + dir + " -name *_depth.txt | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for depth does not exit!" << endl;
        return;
    }
    string depth_file;
    while (getline(fp, depth_file))
        depth_files.push_back(depth_file);
    fp.close();
}