#ifndef GPARSER_H
#define GPARSER_H

//#include <QObject>
//#include <QFile>
//#include <QList>
//#include <QString>

#include <iostream>
#include <string>
#include <fstream>
#include <list>
#include <sstream>
#include <fstream>
#include <vector>
#include "AuboRobotMetaType.h"

using std::list;
using std::string;
using std::cout;
using std::endl;


#define error_point_value 9999999

//绝对坐标系为90 相对坐标系为91
enum Coord_Type{
    Absolute = 90,          //绝对距离模式
    Relative = 91             //相对距离模式
};

enum Move_Type{
    NC_MOVE_NONE = 0,
    NC_MOVE_LINE = 1,
    NC_MOVE_ARC_CW = 2,
    NC_MOVE_ARC_CCW = 3,

    AUBO_MOVE_JOINT = 100,
    AUBO_MOVE_LINE = 101,
    AUBO_MOVE_ARC = 102,
};


struct paramPoint{
    paramPoint():
    x(0), y(0), z(0),speed(99999), g(99999), i(99999), j(99999),
      io_status(aubo_robot_namespace::Io_Status::None),line_number(0),
      orientation{0,0,0,0}
    {}
    double x;
    double y;
    double z;
    double speed;
    int g;
    double i;
    double j;
    aubo_robot_namespace::Io_Status io_status;
    int line_number;
    aubo_robot_namespace::Ori orientation;        // 机械臂姿态信息,四元素(w,x,y,z)
};

class GParser
{

public:
    list<paramPoint> posList;
    explicit GParser();

   Coord_Type get_coord_type();

   void ParsingFile(const string& file_name,list<paramPoint>& List);

//    void filePos(list<paramPoint> List);
    void ParsingFile(const string& file_name);

protected:
    void ClearData();
    bool StringSplit(std::string src, std::string pattern, std::vector<std::string>& strVec);
    double get_g_num(std::string s);
    void parse_g_num(int num);
    paramPoint ParseParam(std::vector<std::string> list); //set parsing line file
    void save_to_last_point(paramPoint & now_point);
    double length_unit_conver = 0.001;
private:

    paramPoint last_point;//上一个坐标相对于原点的位移,即累加和

    Coord_Type this_Coord_Type = Coord_Type::Absolute;
    int Line_Number = 0;

    //nc长度单位到遨博长度单位的换算系数,需要根据g-code参数进行设置
    //默认:nc里的坐标单位国内大多数用mm，aubo为m，需要除以1000

};

#endif // GPARSER_H


