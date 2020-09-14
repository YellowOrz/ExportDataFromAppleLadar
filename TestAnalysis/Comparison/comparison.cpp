#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
using namespace std;
using namespace Eigen;


double PointCloudError(const pcl::PointCloud<pcl::PointXYZRGB> &p1, const pcl::PointCloud<pcl::PointXYZRGB> &p2);
void ICP_BA(const pcl::PointCloud<pcl::PointXYZRGB> &p1, const pcl::PointCloud<pcl::PointXYZRGB> &p2, Isometry3d &transform);
bool ICP_SVD(const pcl::PointCloud<pcl::PointXYZRGB> &p1, const pcl::PointCloud<pcl::PointXYZRGB> &p2, Isometry3d &transform);
bool ReadDepth(double *depth, const string &scence);
bool ReadIntrinsic(Matrix3d &intrinsic, const string &scence);
void GetSceneName(const string &dir, vector<string> &scene);
int main(int argc, char **argv) {
    /* 解析参数，获取第四组数据所在目录 */
    string dir;
    if (argc == 2) dir = argv[1];    // 注意：参数中的路径须以“/”结尾，不是的话程序会自动加上一个
    else {
        cout << RED << "Error: " << RESET << "There is no entering of path of group 4 data!!!" << endl;
        //dir = "/home/orz/Documents/测试数据/4/";
        exit(0);
    }
    if (dir[dir.length() - 1] != '/') {
        cout << RED << "Warning: " << RESET
             << "Direction should end by '/'! But don't worry, the program will add it for you :)" << endl;
        dir += '/';
    }
    /* 遍历场景 */
    vector<string> scene;
    GetSceneName(dir,scene);

    for (int s = 1; s <= scene.size(); s++) {
        cout << BLUE << scene[s] << RESET << endl;
        /* 从文件读取数据 */
        // 读取pcd
        string pcd_path = dir + scene[s] + ".pcd";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile(pcd_path, *cloud_origin)) {
            cout << RED << "Error: " << RESET << pcd_path << " don't exit!!!" << endl;
        } else cout << YELLOW << "Reading: " << RESET << pcd_path << endl;

        // 读取最后一帧的depth
        double depth[256 * 192];  // 深度图大小固定为256*192
        if (!ReadDepth(depth, dir + scene[s])) {
            cout << RED << "Error: " << RESET << " read depth filed!!!" << endl;
        }
        //cout << depth[0] << endl;

        // 读取最后一帧的相机内参
        Matrix3d intrinsic = Matrix3d::Zero();
        if (!ReadIntrinsic(intrinsic, dir + scene[s])) {
            cout << RED << "Error: " << RESET << " read intrinsic filed!!!" << endl;
        }
        //cout << intrinsic << endl;

        /* 从深度图+内参还原点云 */
        cout << YELLOW << "Processing: " << RESET << "recover point cloud from depth." << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_recover(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_recover->width = 256;
        cloud_recover->height = 192;
        cloud_recover->resize(256 * 192);
        //vector<pair<double,double>> errors;
        for (int i = 0; i < cloud_recover->size(); i++) {
            Vector3d pixel(i / 256 * 7.5, (192 - i % 256) * 7.5,
                           1); // 像素坐标系(u, v, 1)。如果不是"192-i%256"而是“i%256”的话，最后的点云与.pcd中读出来的就是镜像关系
            Vector3d image_plane = intrinsic.inverse() * pixel;   // 成像平面坐标系(x', y', 1)
            //cout << image_plane << endl << intrinsic.inverse() << endl;
            cloud_recover->points[i].y = image_plane(0) * depth[i];
            cloud_recover->points[i].x = image_plane(1) * depth[i];
            cloud_recover->points[i].z = depth[i];
            cloud_recover->points[i].r = 200;
            cloud_recover->points[i].g = 200;
            cloud_recover->points[i].b = 200;
            //pair<double,double> error(i, sqrt(pow(p1.points[i].x - cloud_recover.points[i].x, 2) +
            //                                  pow(p1.points[i].y - cloud_recover.points[i].y, 2) +
            //                                  pow(p1.points[i].z - cloud_recover.points[i].z, 2)))
        }


        /* 使用ICP计算两个点云中间的变换矩阵 */
        cout << YELLOW << "Processing: " << RESET << "ICP" << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp_svd(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 手动求解ICP，使用SVD
        Isometry3d transform = Isometry3d::Identity();
        ICP_SVD(*cloud_origin, *cloud_recover, transform); // 计算从cloud_recover变换为cloud_origin的变化矩阵
        pcl::transformPointCloud(*cloud_recover, *cloud_icp_svd, transform.matrix());
        cout << "ICP SVD Error " << PointCloudError(*cloud_origin, *cloud_icp_svd) << endl;

        // PCL库迭代ICP求解。使用shoudong
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud_icp_svd);
        icp.setInputTarget(cloud_origin);
        //icp.setMaxCorrespondenceDistance (3);   // 点对
        icp.setMaximumIterations (1000);  //迭代次数
        icp.setTransformationEpsilon (1e-15);
        icp.setEuclideanFitnessEpsilon(10000); //均方误差（MSE）
        icp.align(*cloud_icp_pcl);
        //cout << "ICP" << endl << icp.getFinalTransformation() << endl;
        cout << "ICP PCL Error " << PointCloudError(*cloud_origin, *cloud_icp_pcl) << endl;
        cout << "ICP PCL Score " << icp.getFitnessScore() << endl;
        //pcl::io::savePCDFileASCII(pcd_path, *cloud_recover_transform);


        /* 多窗口可视化 */
        pcl::visualization::PCLVisualizer::Ptr viewer_all(new pcl::visualization::PCLVisualizer("viewer_all"));
        int v0(0), v1(0), v2(0), v3(0), v4(0);
        viewer_all->createViewPort(0.0, 0.0, 0.333, 1, v0);
        viewer_all->createViewPort(0.333, 0.0, 0.666, 1, v1);
        viewer_all->createViewPort(0.666, 0.0, 1.0, 1, v2);
        viewer_all->addPointCloud(cloud_origin, "cloud_origin_v0", v0);
        viewer_all->addPointCloud(cloud_recover, "cloud_recover_v0", v0);
        viewer_all->addPointCloud(cloud_origin, "cloud_origin_v1", v1);
        viewer_all->addPointCloud(cloud_icp_svd, "cloud_icp_svd", v1);
        viewer_all->addPointCloud(cloud_origin, "cloud_origin_v2", v2);
        viewer_all->addPointCloud(cloud_icp_pcl, "cloud_icp_pcl", v2);

        while (!viewer_all->wasStopped()) {
            viewer_all->spinOnce(100);
        }
    }


    return 0;
}



double PointCloudError(const pcl::PointCloud<pcl::PointXYZRGB> &p1, const pcl::PointCloud<pcl::PointXYZRGB> &p2) {
    double error = 0;
    for (int i = 0; i < p1.size(); i++) {
        error += sqrt(pow(p1.points[i].x - p2.points[i].x, 2) +
                      pow(p1.points[i].y - p2.points[i].y, 2) +
                      pow(p1.points[i].z - p2.points[i].z, 2));
    }
    return error;
}

bool ICP_SVD(const pcl::PointCloud<pcl::PointXYZRGB> &p1,
             const pcl::PointCloud<pcl::PointXYZRGB> &p2,
             Isometry3d &transform) {
    // 计算质心
    Vector3d o = Vector3d::Zero(), r = Vector3d::Zero();
    for (int i = 0; i < p1.size(); i++) {
        o[0] += p1.points[i].x;
        o[1] += p1.points[i].y;
        o[2] += p1.points[i].z;
        r[0] += p2.points[i].x;
        r[1] += p2.points[i].y;
        r[2] += p2.points[i].z;
    }
    o = o / p1.size();
    r = r / p1.size();

    // 将质心挪到原点
    vector<Vector3d> o_shift, r_shift;
    for (int i = 0; i < p1.size(); i++) {
        Vector3d point;
        point[0] = p1.points[i].x - o[0];
        point[1] = p1.points[i].y - o[1];
        point[2] = p1.points[i].z - o[2];
        o_shift.push_back(point);
        point[0] = p2.points[i].x - r[0];
        point[1] = p2.points[i].y - r[1];
        point[2] = p2.points[i].z - r[2];
        r_shift.push_back(point);
    }

    // 计算 W=o*r^T
    Matrix3d W = Matrix3d::Zero();
    for (int i = 0; i < p1.size(); i++) {
        W += Vector3d(o_shift[i](0), o_shift[i](1), o_shift[i](2)) *
             Vector3d(r_shift[i](0), r_shift[i](1), r_shift[i](2)).transpose();
    }

    // SVD分解
    JacobiSVD<Matrix3d> svd(W, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    // 计算变换矩阵
    Matrix3d R = U * V.transpose();
    if (R.determinant() < 0)
        R = -R;
    Vector3d t = o - R * r;
    transform = Isometry3d::Identity();
    transform.rotate(R);
    transform.pretranslate(t);
    //cout << "ICP SVD" << endl << transform.matrix() << endl;
}


bool ReadDepth(double *depth, const string &scence) {
    // 获取路径
    string shell = "find " + scence + " -name *_depth.txt | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for depth does not exit!" << endl;
        return false;
    }
    string depth_path;
    getline(fp, depth_path);
    fp.close();


    // 将depth读到一个数组里面
    fp.open(depth_path.c_str());
    if (!fp) {
        cout << RED << "Error: " << RESET << depth_path << " does not exit!" << endl;
        return false;
    } else cout << YELLOW << "Reading: " << RESET << depth_path << endl;
    string data;
    int i = 0;
    while (getline(fp, data)) {
        depth[i] = -stod(data) / 1000; //文件中单位为mm，这里要转换为m
        i++;
    }
    fp.close();
    return true;
}

bool ReadIntrinsic(Matrix3d &intrinsic, const string &scence) {
    // 获取路径
    string shell = "find " + scence + " -name *_camera.txt | sort -r > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if (!fp) {
        cout << RED << "Error: " << RESET << "filename.txt for intrinsic does not exit!" << endl;
        return false;
    }
    string camera_path;
    getline(fp, camera_path);
    fp.close();


    // 从文件中读取内参的数据到string
    fp.open(camera_path);
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


    // 将string格式的数据转成Matirx
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

    char *strs = new char[data.length() + 1]; // 根据分隔符“,”分割字符串。先转化为char×类型
    strcpy(strs, data.c_str());
    char d = ',';
    char *p = strtok(strs, &d);
    int i = 0;
    while (p) {
        intrinsic(i % 3, i / 3) = stod(p);
        p = strtok(nullptr, &d);
        i++;
    }

    return true;
}

void GetSceneName(const string &dir, vector<string> &scene){
    string shell = "ls -F "+dir+"|grep /|cut -f 1 -d / > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    string data;
    while(getline(fp,data))
        scene.push_back(data);
}