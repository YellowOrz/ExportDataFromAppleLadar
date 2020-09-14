#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <dirent.h>
#include <fstream>

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define YELLOW  "\033[33m"      /* Yellow */
using namespace std;
using namespace cv;
/*递归读取Buffer文件夹下面所有的buffer文件
并且创建好confidence保存的目录*/
bool GetFilesPath(const string& father_dir, vector<string> &files ){
    string shell = "find " + father_dir + " -name *_conf.jpeg | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for PCD does not exit!" << endl;
        return false;
    }
    string file;
    while(getline(fp, file))
        files.push_back(file);
    fp.close();
    return true;
}

int main(int argc, char **argv) {
    /*获取所有要可视化的confidence文件路径，输入参数可以选择文件夹or文件*/
    string path;
    vector<string> conf_files;
    // 解析参数
    if (argc == 2) path = argv[1];    // 注意：参数中的路径必须以“/”结尾！！！
    else {
        cout << RED << "Error: " << RESET << "Please enter a folder path including multiple confidence files or a a single confidence file's path!!!" << endl;
        exit(0);
    }
    // 单个confidence文件可视化
    if(path.find("_conf.jpeg") != string::npos){
        cout << YELLOW << "Visualization Mode" << RESET << ": Single Confidence File" << endl;
        cout << "Confidence is " << path << endl;
        conf_files.push_back(path);
    }
    // 多个confidence文件可视化
    else{
        if (path[path.length() - 1] != '/'){
            cout << RED << "Warning: " << RESET << "Direction should end by '/'! But don't worry, the program will add it for you. :)" << endl;
            path += '/';
        }
        cout << YELLOW << "Visualization Mode" << RESET << ": Multiple Confidence Files" << endl;
        cout << "Folder is " << path << endl;
        GetFilesPath(path, conf_files);
    }
    if (conf_files.empty() ){
        cout << RED << "Error: " << RESET << "Can't find any confidence file!" << endl;
        exit(0);
    }

    /*一个个处理*/
    for (auto & conf_file : conf_files){
        cout << YELLOW << "Reading: " << RESET << conf_file << endl;
        Mat img = imread(conf_file);
        Mat img_RGB = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
        //imshow("img",img);
        //waitKey(0);
        for (int c= 0; c < img.cols; c++) {
            for (int r = 0; r<img.rows;r++){
                int conf = img.at<Vec3b>(r, c)[0];
                img.at<Vec3b>(r, c)[conf] =255;     // 因为opencv中用的是BGR，所以置信度和颜色的关系为0-蓝色，1-绿色，2-红色
            }
        }
        string path = conf_file.erase(conf_file.find("jpeg"))+"jpg";
        imwrite(path,img);
        cout << YELLOW << "Saving: " << RESET << path << endl;
    }
    return 0;
}
