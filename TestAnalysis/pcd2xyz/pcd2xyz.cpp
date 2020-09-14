#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/console/parse.h>
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
using namespace std;

void GetPCDFiles(const string& dir, vector<string> &pcd_files);
int main(int argc, char **argv) {
    /* 解析参数，获取pcd文件名。可以查看单个pcd，也可以查看多个 */
    string path;
    vector<string> pcd_files;
    if (argc == 2) path = argv[1];    // 注意：参数中的路径须以“/”结尾，不是的话程序会自动加上一个
    else {
        cout << RED << "Error: " << RESET << "There is no entering of path of a single pcd or direction of multiple pcd!!!" << endl;
        exit(0);
    }
    if(path.find(".pcd") != string::npos){
        cout << YELLOW << "Transform Mode: " << RESET << "Single" << endl;
        cout << "File is " << path << endl;
        pcd_files.push_back(path);
    }
    else{
        if (path[path.length() - 1] != '/'){
            cout << RED << "Warning: " << RESET << "Direction should end by '/'! But don't worry, the program will add it for you. :)" << endl;
            path += '/';
        }
        cout << YELLOW << "Transform Mode: " << RESET << "Multiple" << endl;
        cout << "Folder is " << path << endl;
        GetPCDFiles(path, pcd_files);
    }

    /* 遍历每一个pcd */
    for(int i=0;i<pcd_files.size();i++){
        /* 读取点云 */
        string pcd_file = pcd_files[i];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(pcl::io::loadPCDFile(pcd_file, *cloud)==-1){
            cout << RED << "Error: " << RESET << pcd_file <<" does not exit!" << endl;
            exit(0);
        }else cout << YELLOW << "Reading: " << RESET << pcd_file << endl;

        /* 保存.xyz */
        string xyz_file = pcd_file.replace(pcd_file.find(".pcd"), 4, ".xyz");
        ofstream fp(xyz_file, ios::app);
        if(!fp.is_open()){
            cout << RED << "Error: " << RESET << xyz_file <<" create failed!" << endl;
            exit(0);
        }else cout << YELLOW << "Saving: " << RESET << xyz_file << endl;

        for(int i = 0;i < cloud->size() ;i++){
            auto points = cloud->points[i];
            fp << points.x << " " << points.y << " " << points.z << " "
                    << int( points.r) << " " << int(points.g) << " " << int(points.b) << endl;
        }
        fp.close();
    }
    return 0;
}

void GetPCDFiles(const string& dir, vector<string> &pcd_files){
    string shell = "find " + dir + " -name *.pcd | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for PCD does not exit!" << endl;
        return;
    }
    string pcd_file;
    while(getline(fp, pcd_file))
        pcd_files.push_back(pcd_file);
    fp.close();
}
