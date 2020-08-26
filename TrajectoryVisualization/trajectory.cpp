#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <iostream>
#include <dirent.h>
#include <unistd.h>
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
using namespace std;
using namespace Eigen;
// 获取所有相机参数的文件路径
bool GetFilesPath(string father_dir, vector<string> &files ){
    DIR *dir=opendir(father_dir.c_str());
    if(dir==NULL){
        cout << father_dir << " don't exit!" << endl;
        return false;
    }
    struct dirent *entry;
    while((entry = readdir(dir)) != NULL){
        string name(entry->d_name);
        if(entry->d_type == DT_DIR ) { // 类型为目录
            if (name.find(".") == string::npos) {	//排除“.”和“..”以及隐藏文件夹
                string son_dir = father_dir + name + "/";

                vector<string> tempPath;
                GetFilesPath(son_dir, tempPath);
                files.insert(files.end(), tempPath.begin(), tempPath.end());
            }
        }
        else if(name.find("camera") != string::npos) // 类型为文件
            files.push_back(father_dir+name);
    }
    closedir(dir);
    return true;
}
// 按文件名称顺序获取所有相机参数的文件路径
bool GetFilesPathInOrder(string father_dir, vector<string> &files ){
    string shell = "ls -l " + father_dir+"|grep camera.txt|awk '{printf $9 \"\\n\"}' > filename.txt";
    system(shell.c_str());
    ifstream fp("filename.txt");
    if(!fp){
        cout << RED << "Error: filename.txt does not exit!" << RESET << endl;
        return false;
    }
    string name="";
    while(getline(fp,name)){
        files.push_back(father_dir+name);
    }
    return true;
}
// 将String中的数字提取出来，放在vector中
void String2Num(string s, vector<double> &nums){
    // 删除字符串中的所有"[]"
    int index = s.find("[");
    while(index != s.npos){
        s.erase(index,1);
        index = s.find("[");
    }
    index = s.find("]");
    while(index != s.npos){
        s.erase(index,1);
        index = s.find("]");
    }
    // 删除空格
    index = s.find(" ");
    while(index != s.npos){
        s.erase(index,1);
        index = s.find(" ");
    }
    // 删除“()”及其前后的字符串
    index = s.find("(");
    s.erase(s.begin(),s.begin()+index+1);
    s.erase(s.end()-1);

    // 根据分隔符“,”分割字符串
    char *strs = new char[s.length() + 1]; // 先转化为char×类型
    strcpy(strs, s.c_str());
    char d=',';
    char *p=strtok(strs,&d);
    while(p){
        nums.push_back(stod(p));
        p = strtok(NULL,&d);
    }

}
// 将vector转成Matrix（动态大小的）
void Vector2Matrix(MatrixXd &m, vector<double> &nums){
    int width = sqrt(nums.size());
    if(nums.size() != width*width){
        cout << RED << "Error: vector nums can't translate to a square matrix, because its size is " << nums.size() << RESET << endl;
        return;
    }
    m=MatrixXd::Zero(width,width);
    for (int y = 0; y < width; y++)
        for (int x = 0; x < width; x++)
            m(x, y) = nums[y*width+x];
}

// 轨迹可视化。来源《视觉SLAM14讲》第三讲
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        // 画出连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(){
    string dir = "../camerafiles/";
    vector<string> files;
    if(!GetFilesPathInOrder(dir, files)){
        exit(0);
    }
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    //从文件中读取想要的矩阵
    for(int i = 0; i<files.size();i++){
        string file = files[i], data;
        cout << file << endl;
        ifstream fp(file);
        if (!fp) {
            cout << RED << "Error: " << "file " << file <<" does not exit!" << RESET << endl;
            continue;
        }
        cout << YELLOW << file << RESET << endl;

        Matrix3d intrinsic;
        Isometry3d view_matrix;
        while(getline(fp,data)){
            //// 提取内参矩阵
            //if(data == "cameraIntrinsics"){
            //    getline(fp,data);   //先将字符串从文件中提取出来
            //    vector<double> nums;
            //    String2Num(data,nums);  //再将字符串中的数字提取出来，数字的数量可能为9也可能为16
            //    MatrixXd temp;
            //    Vector2Matrix(temp,nums);   //最后将数字转为矩阵，大小为3*3或者4*4
            //    intrinsic = temp;
            //}
            // 提取view矩阵
            if(data == "viewMatrixInversed"){
                getline(fp,data);
                vector<double> nums;
                String2Num(data,nums);
                MatrixXd temp;
                Vector2Matrix(temp,nums);
                view_matrix.matrix() = temp;
                poses.push_back(view_matrix);
                //cout<< data<< endl <<view_matrix.matrix() << endl;
            }
        }
        fp.close();
    }
    cout << "poses.size() = " << poses.size() << endl;
    DrawTrajectory(poses);
    return 0;
}