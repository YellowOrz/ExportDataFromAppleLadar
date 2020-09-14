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
bool GetFilesPath(const string& father_dir, vector<string> &files ){
    DIR *dir=opendir(father_dir.c_str());
    if(dir==nullptr){
        cout << father_dir << " don't exit!" << endl;
        return false;
    }
    struct dirent *entry;
    while((entry = readdir(dir)) != nullptr){
        string name(entry->d_name);
        if(entry->d_type == DT_DIR ) { // 类型为目录
            if (name.find('.') == string::npos) {	//排除“.”和“..”以及隐藏文件夹
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
bool GetFilesPathInOrder(const string& father_dir, vector<string> &files ){
    string shell = "find " + father_dir+" -name *camera.txt* | sort > temp.txt";
    system(shell.c_str());
    ifstream fp("temp.txt");
    if(!fp){
        cout << RED << "Error: " << RESET << "filename.txt does not exit!" << endl;
        return false;
    }
    string name="";
    while(getline(fp,name)){
        files.push_back(name);
    }
    fp.close();
    return true;
}
// 将String中的数字提取出来，放在vector中
void String2Num(string s, vector<double> &nums){
    // 删除字符串中的所有"[]"
    int index = s.find('[');
    while(index != string::npos){
        s.erase(index,1);
        index = s.find('[');
    }
    index = s.find(']');
    while(index != string::npos){
        s.erase(index,1);
        index = s.find(']');
    }
    // 删除空格
    index = s.find(' ');
    while(index != string::npos){
        s.erase(index,1);
        index = s.find(' ');
    }
    // 删除“()”及其前后的字符串
    index = s.find('(');
    s.erase(s.begin(),s.begin()+index+1);
    s.erase(s.end()-1);
    // cout << s << endl;

    // // 根据分隔符“,”分割字符串 方法一 纯c++
    // char *strs = new char[s.length() + 1]; // 先转化为char×类型
    // strcpy(strs, s.c_str());
    // char d=',';
    // char *p=strtok(strs,&d);
    // while(p){
    //     nums.push_back(stod(p));
    //     p = strtok(nullptr,&d);
    // }

    //根据分隔符“,”分割字符串 方法二 借助shell
    // string shell = "bash && s="+s+" && b=${s//,/\n} $$ echo $b>temp1.txt";
    string shell = "echo "+s+"|sed  's/,/ /g' > temp.txt";
    system(shell.c_str());
    ifstream fp("temp.txt");
    string data;
    while(fp>>data)
        nums.push_back(stod(data));
    fp.close();
}
// 将vector转成动态大小的Matrix
void Vector2Matrix(MatrixXd &m, vector<double> &nums){
    int width = sqrt(nums.size());

    if(nums.size() != width*width){
        cout << RED << "Error: " << RESET << "vector can't translate to a square matrix, because its size is " << nums.size() << endl;
            for(int i = 0; i<nums.size();i++)
            cout << nums[i] << " ";
        cout << endl;
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
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.001, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        // 起始点单独画，坐标轴颜色不一样
        Vector3d Ow = poses[0].translation();
        Vector3d Xw = poses[0] * (0.05 * Vector3d(1, 0, 0));   // 坐标轴不要太长
        Vector3d Yw = poses[0] * (0.05 * Vector3d(0, 1, 0));
        Vector3d Zw = poses[0] * (0.05 * Vector3d(0, 0, 1));
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 1.0);   //洋红
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Xw[0], Xw[1], Xw[2]);
        glColor3f(1.0, 1.0, 0.0);   //黄色
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Yw[0], Yw[1], Yw[2]);
        glColor3f(0.0, 1.0, 1.0);   //天蓝
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Zw[0], Zw[1], Zw[2]);
        glEnd();
        
        for (size_t i = 1; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴
            Ow = poses[i].translation();
            double length = 0.05;
            if(i!=poses.size()-1){
                double dist2next = (poses[i+1].translation()  - poses[i].translation()).norm();
                double dist2pre = (poses[i-1].translation()  - poses[i].translation()).norm();
                length = ( dist2next + dist2pre ) * 0.5;   // 根据与前后点的距离来设定坐标轴长度
            }
            Xw = poses[i] * (length*0.1 * Vector3d(1, 0, 0));
            Yw = poses[i] * (length * Vector3d(0, 1, 0));       // 绿轴竖直向下（or上？），所以可以稍微长点，其他的要短一点，不然挤到一起看不清
            Zw = poses[i] * (length*0.1 * Vector3d(0, 0, 1));
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

        // 画出连线，rgb依次增长0.2,
        for (size_t i = 0; i < poses.size() -1 ; i++) {
            int k = i%10;
            float r = k>3?0.2:(k+1)*0.2;
            float g = k>6||k<4?0.2:(k-2)*0.2;
            float b = k<7?0.2:(k-5)*0.2;
            // cout << r << " " << g << " " << b << endl;
            glColor3f(r, g, b);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        // 单独画出首尾的连线，紫色
        glColor3f(0.6, 0.0, 1.0);
        glBegin(GL_LINES);
        auto p1 = poses[0], p2 = poses[poses.size() -1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char** argv){
    /*获取所有要可视化的pcd文件路径，输入参数可以选择文件夹or文件*/
    string dir;
    vector<string> pcd_files;
    // 解析参数
    if (argc == 2) dir = argv[1];    // 注意：参数中的路径须以“/”结尾！！！没有的话程序会自动添加
    else {
        cout << RED << "Error: " << RESET << "Please enter a folder path including files of camera!!!" << endl;
        exit(0);
    }
    if (dir[dir.length() - 1] != '/'){
        cout << RED << "Warning: " << RESET << "Direction should end by '/'! But don't worry, the program will add it for you. :)" << endl;
        dir += '/';
    }

    /* 获取所有的相机参数文件路径 */
    vector<string> files;
    if(!GetFilesPathInOrder(dir, files)){
        exit(0);
    }
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;

    /* 从文件中读取想要的矩阵 */
    double distance = 0;
    for(auto & i : files){
        string file = i, data;
        ifstream fp(file);
        if (!fp) {
            cout << RED << "Error: " << "file " << file <<" does not exit!" << RESET << endl;
            continue;
        }
        cout << YELLOW << "Reading: " << RESET << file << endl;

        Matrix3d intrinsic;
        Isometry3d view_matrix;
        while(getline(fp,data)){
            //// 提取内参矩阵，好像没啥用
            //if(data == "cameraIntrinsics"){
            //    getline(fp,data);   //先将字符串从文件中提取出来
            //    vector<double> nums;
            //    String2Num(data,nums);  //再将字符串中的数字提取出来，数字的数量可能为9也可能为16
            //    MatrixXd temp;
            //    Vector2Matrix(temp,nums);   //最后将数字转为矩阵，大小为3*3或者4*4
            //    intrinsic = temp;
            //}

            // 提取view矩阵
            if(data == "viewMatrix"){
                getline(fp,data);
                vector<double> nums;
                String2Num(data,nums);
                MatrixXd temp;
                Vector2Matrix(temp,nums);
                //cout << temp << endl;
                view_matrix.matrix() = temp.inverse();  // 必须求逆，否则结果不对
                poses.push_back(view_matrix);
                if (poses.size()!=1){
                    double dist2pre = (poses[poses.size()-1].translation()  - poses[poses.size()-2].translation()).norm();
                    distance+=dist2pre;
                    cout << "distance to previous one is " << dist2pre << endl; // 输出与前一个点的距离
                }
                //cout<< data<< endl <<view_matrix.matrix() << endl;
                break; // 后面的就不用再读了，节省时间！
            }

        }
        fp.close();
    }
    cout << "The camera moved a total of "<<distance<<"m" << endl;
    cout << "poses.size() = " << poses.size() << endl;

    /* 可视化 */
    DrawTrajectory(poses);
    return 0;
}