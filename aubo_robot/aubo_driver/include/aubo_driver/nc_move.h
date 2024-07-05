#ifndef AUBO_MOVE_H
#define AUBO_MOVE_H



#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>

#include "AuboRobotMetaType.h"
//#include "util.h"

#include "gparser.h"

//    修改点

extern "C" {
#include <stdio.h>
}

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899


class RobotControlServices;
class RobotMoveService;



class Nc_Move
{
public:
    Nc_Move(RobotControlServices *p ,RobotMoveService   *p_move);
    ~Nc_Move();
    void set_thread_status(aubo_robot_namespace::Nc_Move_Mode status);
    int get_thread_status();

#ifdef nc_cir
    void cir_test();
#endif
#if Save_Point
    std::ofstream file;
    std::ofstream file_ik;
#endif
    double line_speed = 0.1;
    double test_accel = 2.0;
/**********************给外部调用，设置相关参数/获取数据*********************************/
    /**************必须设置的属性start*****************/
    //设置用户坐标系
    bool NC_Set_UserCoord(aubo_robot_namespace::CoordCalibrateByJointAngleAndTool Set_UserCoord);
    //设置工具
    bool NC_Set_Tool(aubo_robot_namespace::ToolInEndDesc Set_Tool);
    //设置文件路径
    bool NC_Set_FilePath(const string& Set_FilePath);
    //设置io相关属性
    bool Nc_Set_Io(aubo_robot_namespace::RobotIoType IoType,const string Set_Io, int HighOrLowLevel,bool is_modbus = false);
    /**************必须设置的属性end*****************/

    //设置线速度&加速度,单位为mm
    bool NC_Set_LineSpeed(double Set_LineSpeed,double acc = 50);
    //设置交融半径
    bool Nc_set_ArrivalAheadBlendDistance(double dis);
    //设置缩放系数. ps:不会直接改变生成的点或者轨迹，需要调用者自行调用相应函数生成
    bool Nc_Set_ScaleFactor(double ScaleFactor);
    //设置循环次数以及 循环间隔(单位为s)不设置的话分别默认为1次 2秒
    bool Nc_Set_CyclesAndIntervalTime(int cycles,int IntervalTime);

    //回调的形式获取点的集合
    bool NC_Get_Points_Bycallback();
    //获取点的集合
    bool NC_Get_Points(std::list<aubo_robot_namespace::Nc_Points>& NC_PosList,
                       aubo_robot_namespace::Nc_Point_BaseOrUser BaseOrUser = aubo_robot_namespace::Nc_Point_BaseOrUser::User);
    //获取当前缩放系数
    double Nc_get_ScaleFactor();
    //获取点的集合的size.ps:点位解析未完成时 返回0
    int Nc_Get_PointCount();

    //开始运动
    bool start_cpp_thread(bool block = true, aubo_robot_namespace::MoveModeType type = aubo_robot_namespace::MoveModeType::MOVE_GROUP, int interval = 0 );
    //发送运动指令 (暂停 停止 继续)
    bool Nc_motion_control(aubo_robot_namespace::Nc_Move_Cmd cmd);

    //注册状态回调函数
    void  NcRegisterNcStatusCallbackService(NcStatusCallback ptr, void  *arg);
    //注册获取nc路点集合回调函数
    void  NcRegisterNcGetPointCallbackService(NcGetPointCallback ptr, void  *arg);
    //注册获取nc逆解失败回调函数
    void  NcRegisterNcGetIkErorCallbackService(NcGetIkErorCallback ptr, void  *arg);

    //修改部分点(无实现)
    bool Nc_Change_Points();
    bool NC_Get_InitPoint(aubo_robot_namespace::Nc_Points &point);

    bool Nc_Get_FileType(aubo_robot_namespace::NC_Import_File_Type &fileType);
/**************************************************************************************/



protected:
#ifdef nc_with_qt
    void run();
#endif
    //打印信息(nc内部使用)
    void message_show(string text);

    //发送当前状态线程,回调的函数
    void send_move_status();
    //get_point线程,回调的函数
    void get_point_thread();
    //逆解失败,回调的函数
    void send_IkError(aubo_robot_namespace::Nc_IkError IkError);
    //退出运动
    void exit_motion();


    //运动线程
    void run_cpp_thread();


    //发送io控制指令
    void nc_io_control(int point_id, int status);
    //登录
    bool  aubo_init();
    //复位
    bool revert_machine();
    bool revert_machine_aubo();
    //获取实时机械臂关节状态
//    static void RealTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatusPtr, int size, void *arg);
    //获取实时机械臂事件回调函数
    static void RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg);
    //知道圆上两点以及圆心 求另一个点 并加入list
    bool add_cir_move_to_list( aubo_robot_namespace::Pos pos_1,aubo_robot_namespace::Pos pos_2,aubo_robot_namespace::Pos cir_center_pos
                               ,list<aubo_robot_namespace::Nc_Points> &Nc_points
                               ,aubo_robot_namespace::Pos&  last_pos
                               ,int g_mode = NC_MOVE_ARC_CW
                               ,aubo_robot_namespace::Nc_Point_BaseOrUser BaseOrUser = aubo_robot_namespace::Nc_Point_BaseOrUser::User
                                );
    //知道圆上三点并加入list
        void add_cir_move_to_list(aubo_robot_namespace::Pos pos_1, aubo_robot_namespace::Pos pos_2, aubo_robot_namespace::Pos pos_3,
                                  list<aubo_robot_namespace::Nc_Points> &Nc_points,
                                  aubo_robot_namespace::Pos&  last_pos,
                                  aubo_robot_namespace::Nc_Point_BaseOrUser BaseOrUser = aubo_robot_namespace::Nc_Point_BaseOrUser::User);

#ifdef nc_cir
    //三维
    void add_cir_move_wd3(aubo_robot_namespace::Pos pos_1,aubo_robot_namespace::Pos pos_2,aubo_robot_namespace::Pos cir_center_pos);
#endif
private:
    static Nc_Move* mypointer;

    GParser *gparse = nullptr;
    list<paramPoint> posList;

    bool pos_is_ok = false;
    aubo_robot_namespace::Nc_Move_Mode m_Nc_mode =  aubo_robot_namespace::Nc_Move_Mode::Nc_IS_IDLE;
    aubo_robot_namespace::Nc_Move_Cmd m_Nc_cmd =  aubo_robot_namespace::Nc_Move_Cmd::Default_Nc;
    std::vector<aubo_robot_namespace::Nc_Move_Status> V_nc_move_status;
    std::mutex Nc__move_status_mtx;

    //
    aubo_robot_namespace::NC_Import_File_Type m_importFileType = aubo_robot_namespace::NC_Import_File_Type::NONE;

    //循环次数
    int m_Cycles = 1;
    int m_IntervalTime = 2;

    //Io相关属性
    aubo_robot_namespace::RobotIoType m_IoType;
    string m_Set_Io;
    int m_HighOrLowLevel;

    //设置工具
    aubo_robot_namespace::ToolInEndDesc m_tool;
    //设置用户坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool m_user_coord;
    //p_init 在 user_coord 下的姿态
    aubo_robot_namespace::Ori m_pinit_ori_user;
    //当前路点
    aubo_robot_namespace::wayPoint_S m_init_wp;
    //交融半径
    double Blend_Distance = 0.04;
    //整体缩放系数，默认为1
    double Scale_factor = 1.0;
    //callback
    bool Is_GetPointBycallback = false;
    //
    int wp_id = 0;
    int m_line_number = 0;
    //点位解析完毕的标志
    bool m_PointIk_is_done = false;
    int    m_point_count = 0;
    //
    bool m_io_is_modbus = false;
    //move_group param
    aubo_robot_namespace::MoveModeType m_movegroup_type = aubo_robot_namespace::MoveModeType::MOVE_GROUP;
    int m_movegroup_interval = 0;

    NcStatusCallback                      m_NcStatusCallback = NULL;
    void                                     *m_NcStatusCallbackArg = NULL;

    NcGetPointCallback                      m_NcGetPointCallback = NULL;
    void                                     *m_NcGetPointCallbackArg = NULL;

    NcGetIkErorCallback             m_NcGetIkErorCallback = NULL;
    void                                     *m_NcGetIkErorCallbackArg = NULL;

    RobotControlServices            *m_robotBaseService;          //机械臂基础服务
    RobotMoveService                 *m_robotMoveService;
};

#endif // AUBO_MOVE_H
