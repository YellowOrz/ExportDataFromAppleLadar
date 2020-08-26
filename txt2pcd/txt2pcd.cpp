#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define YELLOW  "\033[33m"      /* Yellow */
using namespace std;

//根据分隔符分割字符串，然后存入vector
// https://blog.csdn.net/Mary19920410/article/details/77372828
vector<string> split(const string &str, const string &delim) {
    vector<string> res;
    if ("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型
    char *strs = new char[str.length() + 1]; //不要忘了
    strcpy(strs, str.c_str());

    char *d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
        string s = p; //分割得到的字符串转换为string类型
        res.push_back(s); //存入结果数组
        p = strtok(NULL, d);
    }
    return res;
}

/*递归读取Buffer文件夹下面所有的buffer文件
并且创建好pcd保存的目录*/
bool GetFilesPath(string father_dir, vector<string> &files ){
    DIR *dir=opendir(father_dir.c_str());
    if(dir==NULL){
        cout << RED << "Error: " << RESET << "dir " << father_dir << " don't exit!" << endl;
        return false;
    }
    struct dirent *entry;
    while((entry = readdir(dir)) != NULL){
        string name(entry->d_name);
        // 类型为目录                    排除“.”和“..”以及隐藏文件夹
        if(entry->d_type == DT_DIR ) {
            if (name.find(".") == string::npos) {
                string son_dir = father_dir + name + "/";

                //创建png的目录
                string png_son_dir = son_dir;
                string mkdir = "mkdir -p " + png_son_dir.replace(png_son_dir.find("txt"), 3, "pcd");
                //cout << mkdir << endl;
                system(mkdir.c_str());

                vector<string> tempPath;
                GetFilesPath(son_dir, tempPath);
                files.insert(files.end(), tempPath.begin(), tempPath.end());
            }
        }
        // 类型为文件。只找buffer文件
        else if(name.find("txt") != string::npos)
            files.push_back(father_dir + name);
    }
    closedir(dir);
    return true;
}

void Buffer2PCD(string filepath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
    // 打开文件
    ifstream buffer(filepath);
    if (!buffer) {
        cout << RED << "Error: " << RESET << "file " << filepath <<" does not exit!" << endl;
        exit(0);
    }
    else cout << YELLOW << "Reading: " << RESET << filepath << endl;
    //将文件中的输入存入pcl点云
    string data;

    //排除第一行
    getline(buffer, data);

    while (getline(buffer, data)) {
        // 排除空点
        if (data.find("0;0;0;0;0;0;0") != string::npos)
            break; // 还是continue比较好？

        // 字符串分割
        vector<string> datas = split(data, ";");

        // 把数据存入三维点中
        pcl::PointXYZRGB point;
        point.x = stof(datas[1]);
        point.y = stof(datas[2]);
        point.z = stof(datas[3]);
        int temp = int(stof(datas[4]) * 250);
        point.r = temp > 255 ? 255 : temp;
        temp = int(stof(datas[5]) * 250);
        point.g = temp > 255 ? 255 : temp;
        temp = int(stof(datas[6]) * 250);
        point.b = temp > 255 ? 255 : temp;
        //point.label = stoi(datas[7]);

        // 加入点云中
        cloud->push_back(point);
    }
    buffer.close();
}

int
main() {
    // 获取所有要处理的文件路径
    string buffer_dir = "../txt/";
    vector<string> buffer_files;
    GetFilesPath(buffer_dir,buffer_files);
    if (buffer_files.size() == 0 ){
        cout << RED << "Error: " << RESET << "Can't find any .txt file!" << endl;
        exit(0);
    }

    //一个个处理
    for (int i = 0;i<buffer_files.size();i++){
        //初始化空的点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 读取数据到点云
        Buffer2PCD(buffer_files[i],cloud);

        // 点云可视化
        //pcl::visualization::CloudViewer viewer("Buffer");
        //viewer.showCloud(cloud);
        //while (!viewer.wasStopped()) {}

        // 保存点云
        string path=buffer_files[i].replace(buffer_files[i].find("txt"),3,"pcd");
        path=path.replace(path.find("txt"),3,"pcd"); // 将路径中的两个“txt”都替换成“pcd”
        cout <<YELLOW << "Saving: " << RESET <<  path << endl;
        pcl::io::savePCDFileASCII(path, *cloud);
    }
    return 0;
}
