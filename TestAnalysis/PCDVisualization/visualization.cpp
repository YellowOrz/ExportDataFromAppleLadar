#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/console/parse.h>
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define YELLOW  "\033[33m"      /* Yellow */
using namespace std;

/*递归读取Buffer文件夹下面所有的buffer文件
并且创建好pcd保存的目录*/
bool GetFilesPath(const string& father_dir, vector<string> &files ){
    DIR *dir=opendir(father_dir.c_str());
    if(dir==nullptr){
        cout << RED << "Error: " << RESET << "dir " << father_dir << " don't exit!" << endl;
        return false;
    }
    struct dirent *entry;
    while((entry = readdir(dir)) != nullptr){
        string name(entry->d_name);
        // 类型为目录                    排除“.”和“..”以及隐藏文件夹
        if(entry->d_type == DT_DIR ) {
            if (name.find('.') == string::npos) {
                string son_dir = father_dir + name + "/";

                vector<string> tempPath;
                GetFilesPath(son_dir, tempPath);
                files.insert(files.end(), tempPath.begin(), tempPath.end());
            }
        }
            // 类型为文件。只找pcd格式的点云文件
        else if(name.find("pcd") != string::npos)
            files.push_back(father_dir + name);
    }
    closedir(dir);
    return true;
}



int main(int argc, char **argv) {

    string path;
    vector<string> pcd_files;
    /* 输入参数只有一个。对应的是浏览单个pcd or 依次浏览多个pcd*/
    if (argc == 1){
        cout << RED << "Error: " << RESET << "Please input single pcd's path or multiple pcd's path or direction including pcd" << endl;
        cout << "For example: " << endl << "\t\t./PCDVisualization test.pcd"
                                << endl << "\t\t./PCDVisualization test1.pcd test2.pcd"
                                << endl << "\t\t./PCDVisualization ./test/" << endl;
    }
    else if (argc == 2) {
        path = argv[1];    // 注意：参数中的路径必须以“/”结尾！！！
        // 单个PCD文件可视化
        if(path.find(".pcd") != string::npos){
            cout << YELLOW << "Visualization Mode" << RESET << ": Single PCD File" << endl;
            cout << "PCD is " << path << endl;
            pcd_files.push_back(path);
        }
            // 多个PCD文件可视化
        else{
            if (path[path.length() - 1] != '/'){
                cout << RED << "Warning: " << RESET << "Direction should end by '/'! But don't worry, the program will add it for you. :)" << endl;
                path += '/';
            }
            cout << YELLOW << "Visualization Mode" << RESET << ": Multiple PCD Files In Turn" << endl;
            cout << "Folder is " << path << endl;
            GetFilesPath(path, pcd_files);
        }
        if (pcd_files.empty() ){
            cout << RED << "Error: " << RESET << "Can't find any .pcd file!" << endl;
            exit(0);
        }

        /*一个个处理*/
        for (auto & pcd_file : pcd_files){
            //初始化空的点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            //读取点云
            if (pcl::io::loadPCDFile(pcd_file,*cloud) == -1) {
                cout << RED << "Error: " << RESET << "file " << pcd_file <<" does not exit!" << endl;
                exit(0);
            }
            else cout << YELLOW << "Reading: " << RESET << pcd_file << endl;

            //窗口背景颜色根据点云颜色自适应
            double r=0,g=0,b=0;     // 默认背景为黑色
            int num = 1000, space = cloud->size()/num;
            for(int i = 0;i<cloud->size();i+=space){   //取100个点的平均颜色作为判断标准
                r+=cloud->points[i].r;
                g+=cloud->points[i].g;
                b+=cloud->points[i].b;
            }
            r/=num,g/=num,b/=num;
            if(r<50 && g<50 && b<50) // 如果整体颜色偏黑，则背景颜色为白色
                r=225,g=225,b=225;
            else                    // 否则为黑色
                r=30,g=30,b=30;

            // 点云可视化
            cout <<YELLOW << "Visual: " << RESET <<  pcd_file << endl;
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(pcd_file));
            viewer->setBackgroundColor(r,g,b);
            viewer->addPointCloud(cloud);

            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
            }
        }
    }

    /* 输入多个参数，都为pcd，同时显示。最多支持显示6个 */
    else if(argc <= 7){
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
        cout << YELLOW << "Visualization Mode" << RESET << ": Multiple PCD Files Together " << endl;
        for(int i = 1;i<argc;i++){
            path = argv[i];
            if(path.find(".pcd") == string::npos){
                cout << RED << "Error: " << RESET << path <<" is not a pcd!!!" << endl;
                exit(0);
            }else cout << YELLOW << "Reading: " << RESET << path << endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if(pcl::io::loadPCDFile(path, *cloud) == -1){
                cout << RED << "Error: " << RESET << "file " << path <<" does not exit!" << endl;
                exit(0);
            }
            clouds.push_back(cloud);
        }
        // 窗口被划分为6个，次序为
        // 1 3 5
        // 0 2 4
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View Together"));
        int v[6]={0};
        float h = 1.0/2, w = 1.0/int(argc/2);   // 单个窗口的高、宽。整个窗口高1宽1
        for(int i = 0;i<argc-1;i++){
            //cout <<h<<" "<<w<<" "<< int(i/2)*w << " " << int(i%2)*h << " " << w+int(i/2)*w << " " << h+int(i%2)*h << " " << v[i] << " "<< i << endl;
            viewer->createViewPort(int(i/2)*w, int(i%2)*h, w+int(i/2)*w, h+int(i%2)*h, v[i]);
            viewer->addPointCloud(clouds[i], "cloud"+to_string(i), v[i]);

            //窗口背景颜色根据点云颜色自适应
            double r=0,g=0,b=0;     // 默认背景为黑色
            int num = 1000, space = clouds[i]->size()/num;
            for(int j = 0;j<clouds[i]->size();j+=space){   //取num个点的平均颜色作为判断标准
                r+=clouds[i]->points[j].r;
                g+=clouds[i]->points[j].g;
                b+=clouds[i]->points[j].b;
            }
            r/=num,g/=num,b/=num;
            if(r<50 && g<50 && b<50) // 如果整体颜色偏黑，则背景颜色为白色
                r=225,g=225,b=225;
            else                    // 否则为黑色
                r=30,g=30,b=30;
            viewer->setBackgroundColor(r,g,b,v[i]);
        }
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }
    else{
        cout << RED << "Error: " << RESET << "Input to much paramaters, only support to view 6 pcd at the same time" << endl;
    }

    return 0;
}
