#include <algorithm>            // std::min, std::max
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>  
#include <time.h>
#include <math.h>
#include "..\socket\tcpsocket.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <Ewma.h>


using namespace std;

#define ENableURtoMOVE          1
#define ENableAGtoMOVE          0
#define EnableShowDebug         0 //debug���

#define ATI_IP "192.168.1.1"  //ATI������ip
#define ATI_Port 49152        //ATI�˿�
#define UR_Port					30003  //8ms  ��Ҫͨ�� 500hz
#define UR_IPaddr				"192.168.1.52" //UR������ip��ַ
#define SendMAXlen              1024  // ����������󳤶�
#define RecvMAXlen              1116  // ����������󳤶�
#define q_err                   0.00001//0.0008 ���ֵ

using namespace ur_rtde;
using namespace std::chrono;

char sendBuf[SendMAXlen];
unsigned char recvBuf[2000] = "";

int timeCnt = 0;
int Calcnt = 1;
int ControlFreq = 10; //����Ƶ��
float SwabForce = 0;
extern double DeltaR[5];
extern double ForceInt[5];
float SwabCor = 0;
int iRows = 0;                                        //CSV�ļ��е�����
int OutFileOnce = 0;
int SendMoveTime = 1;
int SendingMoveL = 0;
int ForceOut = 0;
int PosOut = 0;
int filterOut = 0;

//ATI������
double ContactForce[6];

//���Ʋ���
struct ImpedanceControllerParams {
    double mass; //����
    double damping;//����
	double stiffness;//�ն�
    double stiffness_0;//�ն�
	
};

double smoothedValue;
//ָ����Ȩ�ƶ�ƽ���˲���
class ExponentialMovingAverage {
private:
    double alpha; // ָ����Ȩƽ��ϵ��
    double smoothedValue; // ƽ�������ֵ

public:
    // ���캯������ʼ��Ȩ�غͳ�ʼֵ
    ExponentialMovingAverage(double initial, double alpha) : smoothedValue(initial), alpha(alpha) {}

    // ����ƽ�����ֵ
    void update(double measurement) {
        smoothedValue = alpha * measurement + (1 - alpha) * smoothedValue;
    }

    // ��ȡ��ǰƽ�����ֵ
    double getSmoothedValue() const {
        return smoothedValue;
    }
};

#define PI  3.14159265358979323846

double Start_Postion_x; //��ʼλ��
double Start_Postion_y;
double Start_Postion_z;
double Start_Postion_rx;
double Start_Postion_ry;
double Start_Postion_rz;

double Current_Postion_x; //��ǰλ��
double Current_Postion_y; 
double Current_Postion_z; 
double Current_Postion_rx;
double Current_Postion_ry;
double Current_Postion_rz; 

double Target_Postion_x; //Ŀ��λ��
double Target_Postion_y;
double Target_Postion_z;
double Target_Postion_rx;
double Target_Postion_ry;
double Target_Postion_rz;

std::vector<double> Tcp_Pose;
double My_force_controller(double Target_Force, double Current_Force, double Target_Pos, double Start_pos, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Start_Postion_z;

    target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)-params.stiffness * (Current_Position - Target_Pos)) / params.mass;
    current_speed = current_speed + target_acc * dt;
    Current_Position = Current_Position + current_speed * dt;

    return Current_Position;//���ؾ���λ��
}

double My_follow_pos_controller(double Target_Force, double Current_Force, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Start_Postion_z;

    target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    current_speed = current_speed + target_acc * dt;
    Current_Position = Current_Position + current_speed * dt;

    return Current_Position;//���ؾ���λ��
}

double My_follow_pos_controller_rx(double Target_Force, double Current_Force, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double angle_change = 0.0; static double current_speed = 0.0; static double Current_angle = Start_Postion_rx;

    angle_change = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    Current_angle = Current_angle + angle_change;
    cout << Current_angle << endl;

    return Current_angle;//���ؾ���λ��
}

double My_follow_pos_controller_ry(double Target_Force, double Current_Force, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double angle_change = 0.0; static double current_speed = 0.0; static double Current_angle = Start_Postion_ry;

    angle_change = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    Current_angle = Current_angle + angle_change;
    cout << Current_angle << endl;

    return Current_angle;//���ؾ���λ��
}

double My_follow_pos_controller_rz(double Target_Force, double Current_Force, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double angle_change = 0.0; static double current_speed = 0.0; static double Current_angle = Start_Postion_rz;

    angle_change = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    Current_angle = Current_angle + angle_change;
    cout << Current_angle << endl;

    return Current_angle;//���ؾ���λ��
}

double My_force_contact_controller(double Target_Force, double Current_Force, double Target_Pos, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Target_Postion_y;
    Target_Pos = Current_Position;
	cout << "target_position_z: " << Target_Pos << endl;
	target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)-params.stiffness * (Current_Position - Target_Pos)) / params.mass;
	current_speed = current_speed + target_acc * dt;
	Current_Position = Current_Position + current_speed * dt;

	return Current_Position;//���ؾ���λ��
}

double My_force_vel_controller(double Target_Force, double Current_Force, double Target_Pos, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Target_Postion_y;
    Target_Pos = Current_Position;
    cout << "target_position_z: " << Target_Pos << endl;
    target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)-params.stiffness * (Current_Position - Target_Pos)) / params.mass;
    current_speed = current_speed + target_acc * dt;

    return current_speed;//���ؾ����ٶ�
}

double My_force_pos_undamping_controller(double Target_Force, double Current_Force, double Target_Pos, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Target_Postion_z;

    target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    current_speed = current_speed + target_acc * dt;
    Current_Position = Current_Position + current_speed * dt;

    return Current_Position;//���ؾ���λ��
}

double My_force_vel_undamping_controller(double Target_Force, double Current_Force, double Target_Pos, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Start_Postion_y;
    Target_Pos = Current_Position;
    cout << "target_position_z: " << Target_Pos << endl;
    target_acc = ((Current_Force - Target_Force) - params.damping * (current_speed)) / params.mass;
    current_speed = current_speed + target_acc * dt;

    return current_speed;//���ؾ����ٶ�
}

//��������Ӧ
double My_adp_force_controller(double Target_Force, double Current_Force, double Target_Pos, double Error_force, double ki, const static ImpedanceControllerParams& params)
{
    static double dt = 0.09;
    static double target_acc = 0.0; static double current_speed = 0.0; static double Current_Position = Start_Postion_y; static double efk_i = 0.0;

    efk_i = Error_force + dt * Error_force;
    target_acc = ((Current_Force - Target_Force) + ki * efk_i - params.damping * (current_speed)-params.stiffness * (Current_Position - Target_Pos)) / params.mass;
    current_speed = current_speed + target_acc * dt;
    Current_Position = Current_Position + current_speed * dt;

    return Current_Position;//���ؾ���λ��
}

void ReadPos(fstream& InFile)
{
    if (PosOut == 0)  //��������
    {
        PosOut = 1;
        InFile << "X" << "," << "Y" << "," << "Z" << "," << "Rx" << "," << "Ry" << "," << "Rz" << endl;
    }
    else
    {
        InFile << Tcp_Pose[0] << "," << Tcp_Pose[1] << "," << Tcp_Pose[2] << "," << Tcp_Pose[3] << "," << Tcp_Pose[4] << "," << Tcp_Pose[5] << endl;
    }
}

void Readfilter(fstream& InFile)
{
    if (filterOut == 0)  //��������
    {
        filterOut = 1;
        InFile << "X" << "," << "Y" << "," << "Z" << "," << "Rx" << "," << "Ry" << "," << "Rz" << endl;
    }
    else
    {
        InFile << ContactForce[0] << "," << ContactForce[1] << "," << smoothedValue << "," << ContactForce[3] << "," << ContactForce[4] << "," << ContactForce[5] << endl;
    }
}

int main() 
{
    std::string  strForceFileName = "E:\\myProject\\UR_control\\Force.csv";  //�ļ�·��
    std::fstream Forcefile;                                     //����һ���ļ��������������
    Forcefile.open(strForceFileName.c_str(), std::ios::ate | ios::out);           //��д�ļ��ķ�ʽ���ļ������û���ļ��򴴽��ļ�������У�������ļ�

    std::string  strPosFileName = "E:\\myProject\\UR_control\\Pos.csv";  //�ļ�·��
    std::fstream Posfile;                                     //����һ���ļ��������������
    Posfile.open(strPosFileName.c_str(), std::ios::ate | ios::out);           //��д�ļ��ķ�ʽ���ļ������û���ļ��򴴽��ļ�������У�������ļ�

    std::string  strfilterFileName = "E:\\myProject\\UR_control\\filter.csv";  //�ļ�·��
    std::fstream filterfile;                                     //����һ���ļ��������������
    filterfile.open(strfilterFileName.c_str(), std::ios::ate | ios::out);

    RTDEControlInterface rtde_control(UR_IPaddr);
    RTDEReceiveInterface rtde_receive(UR_IPaddr);

    if (rtde_control.isConnected() && rtde_receive.isConnected())
    {
        cout << "Connected to the robot" << endl;
    }
    else
    {
        cout << "Disconnected to the robot" << endl;
    }

    // ���õ��ɿ���������
    ImpedanceControllerParams controller_params;
    controller_params.mass = 35.0;
    controller_params.damping = 550.0;
    controller_params.stiffness = 1000.0;
    controller_params.stiffness_0 = 0.0;
    
    

    ImpedanceControllerParams controller_params1;
    controller_params1.mass = 20.0;
    controller_params1.damping = 550.0;
    //controller_params1.stiffness = 150.0;
    controller_params1.stiffness_0 = 0.0;


    ImpedanceControllerParams controller_params2;
    controller_params2.mass = 35.0;
    controller_params2.damping = 650.0;
    controller_params2.stiffness = 800.0;
    controller_params2.stiffness_0 = 0.0;

    // ��ʼ���˲�������ʼֵΪ0��ָ����Ȩƽ��ϵ��Ϊ0.1
    ExponentialMovingAverage(0, 0.9);

    Tcp_Pose = rtde_receive.getActualTCPPose(); //��ȡ������ʼλ��

    //��������ʼλ�� movel
    //rtde_control.moveJ(StartPoint);
    rtde_control.moveL(Tcp_Pose);

    Start_Postion_x = Tcp_Pose[0];
    Start_Postion_y = Tcp_Pose[1];
    Start_Postion_z = Tcp_Pose[2];
    Start_Postion_rx = Tcp_Pose[3];
    Start_Postion_ry = Tcp_Pose[4];
    Start_Postion_rz = Tcp_Pose[5];

    Current_Postion_x = Start_Postion_x; 
    Current_Postion_y = Start_Postion_y;
    Current_Postion_z = Start_Postion_z;
    Current_Postion_rx = Start_Postion_rx;
    Current_Postion_ry = Start_Postion_ry;
    Current_Postion_rz = Start_Postion_rz;

    Target_Postion_x = Current_Postion_x; 
    Target_Postion_y = Current_Postion_y;
    Target_Postion_z = Current_Postion_z;
    Target_Postion_rx = Current_Postion_rx;
    Target_Postion_ry = Current_Postion_ry;
    Target_Postion_rz = Current_Postion_rz;



    ExponentialMovingAverage filter(0, 0.1);
    ReadATIForce(ATI_IP, ATI_Port, 0, Forcefile); //��ʼ��
    setBias(); //�����������ݹ������
    cout << ContactForce[0] << ContactForce[1] << ContactForce[2] << ContactForce[3] << ContactForce[4] << ContactForce[5] << endl;
    vector<double> data;
    int i = 0;
    
    while (true)
    {
        double desiredForce = 3 + sin(PI * i / 15); // ʾ���е�������
        cout << "������������"<<desiredForce << endl;
        //��������
        Tcp_Pose = rtde_receive.getActualTCPPose();
        ReadPos(Posfile); //��¼λ�ò���
        Current_Postion_x = Tcp_Pose[0];
        Current_Postion_y = Tcp_Pose[1];
        Current_Postion_z = Tcp_Pose[2];
        Current_Postion_rx = Tcp_Pose[3];
        Current_Postion_ry = Tcp_Pose[4];
        Current_Postion_rz = Tcp_Pose[5];
        cout << "��ǰλ��Ϊ��" << Tcp_Pose[0]*1000 << Tcp_Pose[1] * 1000 << Tcp_Pose[2] * 1000 << Tcp_Pose[3] << Tcp_Pose[4] << Tcp_Pose[5] << endl;
        ReadATIForce(ATI_IP, ATI_Port, 1, Forcefile);
        double Current_Force_x = ContactForce[0];
        double Current_Force_y = ContactForce[1];
        double Current_Force_z = ContactForce[2];
        double Current_Force_rx = ContactForce[3];
        double Current_Force_ry = ContactForce[4];
        double Current_Force_rz = ContactForce[5];

        //��Ҫ��һ�������ж����ݽ��д洢����ÿ��ѭ���У�ͨ��������ֵ������������������е����ݽ����˲�������ʵʱ���
        data.push_back(ContactForce[2]);
        //�˲���
        cout << "��һ�Σ�" << ContactForce[2] << endl;

        filter.update(data[i]);
        smoothedValue = filter.getSmoothedValue();
        i++;
        cout << "filter.getSmoothedValue��" << filter.getSmoothedValue() << endl;
        Readfilter(filterfile); //��¼λ�ò���

        //Target_Postion_z = My_force_controller(0, -Current_Force, Start_Postion_z, 0, {controller_params.mass,controller_params.damping,controller_params .stiffness}); //�㶨λ�ÿ���
        //Target_Postion_z = My_follow_pos_controller(0, -Current_Force_z, { controller_params.mass,controller_params.damping,controller_params.stiffness_0 }); //�涯λ�ÿ���
        //Target_Postion_rx = My_follow_pos_controller_rx(0, Current_Force_rx, { controller_params.mass,controller_params.damping,controller_params.stiffness_0 }); //�涯λ�ÿ���
        //Target_Postion_ry = My_follow_pos_controller_ry(0, Current_Force_ry, { controller_params.mass,controller_params.damping,controller_params.stiffness_0 }); //�涯λ�ÿ���
        //Target_Postion_rz = My_follow_pos_controller_rz(0, Current_Force_rz, { controller_params.mass,controller_params.damping,controller_params .stiffness_0}); //�涯λ�ÿ���
        
        cout << "����������" << i << endl;
        //Target_Postion_y = My_force_contact_controller(6, -Current_Force_z, Target_Postion_y, { controller_params2.mass,controller_params2.damping,controller_params2.stiffness_0 }); //����λ�ÿ���
        //Target_Postion_y = My_adp_force_controller(6, -Current_Force_z, Target_Postion_y, 6 + Current_Force_z, 0.5, { controller_params2.mass,controller_params2.damping,controller_params2.stiffness_0 });
        Target_Postion_y = My_force_contact_controller(desiredForce, -Current_Force_z, Target_Postion_y, { controller_params.mass,controller_params.damping,controller_params.stiffness_0 }); //����λ�ÿ���
        //double current_speed = My_force_vel_controller(6, -Current_Force_z, Target_Postion_y, { controller_params.mass,controller_params.damping }); //�����ٶȿ���
        //Target_Postion_z = My_force_pos_undamping_controller(3, -Current_Force, Target_Postion_z, controller_params); //�޸նȺ㶨λ�ÿ���
        //double current_speed = My_force_vel_undamping_controller(3, -Current_Force, Target_Postion_z, controller_params); //�޸նȺ㶨�ٶȿ���
        //cout << Target_Postion_z << endl;
        //cout << Target_Postion_rx << endl;
        //cout << Target_Postion_ry << endl;
        //cout << Target_Postion_rz << endl;
        //cout << current_speed << endl;

        //��;����
        //if (Target_Postion_z-Start_Postion_z > 40*0.001)
        //{
        //    cout << "�������һ������" << endl;
        //    system("pause");
        //    break;
        //}

        Tcp_Pose[0] = Target_Postion_x;
        Tcp_Pose[1] = Target_Postion_y;
        Tcp_Pose[2] = Target_Postion_z;
        Tcp_Pose[3] = Target_Postion_rx;
        Tcp_Pose[4] = Target_Postion_ry;
        Tcp_Pose[5] = Target_Postion_rz;
        cout << "��һλ��Ϊ��" << Tcp_Pose[0] << Tcp_Pose[1] << Tcp_Pose[2] << Tcp_Pose[3] << Tcp_Pose[4] << Tcp_Pose[5] << endl;
        rtde_control.servoL(Tcp_Pose, 0, 0, 0.09, 0.09, 1000);

        //rtde_control.speedL({ 0,current_speed, 0,0,0,0 });

        Sleep(90); //��λ�ÿ���ʱ����˯�ߴ���
    }

    //������ʼλ��
    //rtde_control.servoL({ Start_Postion_x ,Start_Postion_y ,Start_Postion_z ,Start_Postion_rx ,Start_Postion_ry ,Start_Postion_rz }, 0, 0, 3, 0.09, 1000);
    return 0;
}