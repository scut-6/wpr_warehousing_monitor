#include "MissionInfoWidget.h"

static const double Pi = 3.14159265358979323846264338327950288419717;

using namespace std;

CMissionInfoWidget::CMissionInfoWidget(QWidget *parent)
    : QWidget(parent)
{
    // resize(1024,960);
}

CMissionInfoWidget::~CMissionInfoWidget()
{
    
}

inline float twodec(float n) { return floor(n * 100 + 0.5) / 100; }

void CMissionInfoWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    QRect rect(0, 0, width(), height());
	painter.setViewport(rect);
	painter.setWindow(0, 0, width(), height());	//设置窗口大小，逻辑坐标
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::TextAntialiasing);

    QBrush brBg;
	brBg.setColor(Qt::blue);
	brBg.setStyle(Qt::SolidPattern);
	painter.setBrush(brBg);
    painter.drawRect(rect);	

    /*************************** [一] 任务列表 ****************************/
    // 标题
    QFont fontTitle;
    fontTitle.setPointSize(18);
    painter.setFont(fontTitle);
    QPen penTitle;
	penTitle.setColor(Qt::yellow);
	painter.setPen(penTitle);
    // QFontMetrics fm = painter.fontMetrics();
    QString strMissionTitle = QString("当前任务列表");
    // QRect rcText = fm.boundingRect(strMissionTitle);		//得到字符串的Rect
    painter.drawText(5, 25, strMissionTitle);

    // 任务模块
    list<stMission>::iterator it;
    int nMCount = 0;
    int nBlockHeight = 130;
    QPen penText;
    QRect rcMission(0,0,240,200);
    stringstream stringStream;
    QFont fontText;
    fontText.setPointSize(12);
    painter.setFont(fontText);
    for(it = pMisMgr->mission_list.begin();it!=pMisMgr->mission_list.end();it++)
    {
        int x_offset = 15 + nMCount*250;
        int y_offset = 45;
        rcMission.moveTo(x_offset-10,y_offset-15);
        // 画群体任务矩形背景
        int mission_state = (*it).state;
        int nTCount = (*it).arTask.size();
        if(mission_state == M_ST_IN_PROGRESS)
        {
            brBg.setColor(QColor(0, 155, 0));
            penText.setColor(QColor(255, 255, 255));
        }
        else
        {
            brBg.setColor(QColor(100, 100, 100));
            penText.setColor(QColor(150, 150, 150));
        }
        rcMission.setHeight(50+ nBlockHeight*nTCount);
	    painter.setBrush(brBg);
        painter.drawRect(rcMission);	
	    painter.setPen(penText);
        // [1] 群体任务名称
        stringStream.str("");
        stringStream << "[任务] " <<  (*it).mission;
		string mission_name =  stringStream.str();
        painter.drawText(x_offset,y_offset+5, QString::fromStdString(mission_name));
        stringStream.str("");
        stringStream << "[状态] " << str_helper.GetMissionStateStr(mission_state);
        string m_state_str = stringStream.str();
        painter.drawText(x_offset,y_offset+25, QString::fromStdString(m_state_str));
        // [2] 单体任务列表
        for(int i=0;i<nTCount;i++)
        {
            // 画单体任务矩形背景
            QRect rcTask(0,0,230,120);
            rcTask.moveTo(x_offset-5,y_offset + nBlockHeight*i+35);
            if(mission_state == M_ST_IN_PROGRESS)
            {
                brBg.setColor(QColor(0, 0, 100));
            }
            else
            {
                brBg.setColor(QColor(70, 70, 70));
            }
            painter.setBrush(brBg);
            painter.drawRect(rcTask);	
            
            // [2-1] 单体任务机器人ID
            stringStream.str("");
            int robot_id = (*it).arTask[i].command.id;
            stringStream << "执行者ID = " << robot_id;
            string id_str = stringStream.str();
            painter.drawText(x_offset,y_offset + 60 + nBlockHeight*i, QString::fromStdString(id_str));
            // [2-2] 单体任务指令
            stringStream.str("");
            int cmd = (*it).arTask[i].command.command;
            stringStream << "分配指令 " << str_helper.GetCommandStr(cmd);
            string cmd_str = stringStream.str();
            painter.drawText(x_offset,y_offset + 60 + nBlockHeight*i + 20, QString::fromStdString(cmd_str));
            // [2-3] 单体任务坐标
            stringStream.str("");
            float map_x = (*it).arTask[i].command.map_x;
            stringStream << "map_x= " << twodec(map_x);
            string map_x_str = stringStream.str();
            painter.drawText(x_offset,y_offset + 60 + nBlockHeight*i + 20* 2, QString::fromStdString(map_x_str));
            stringStream.str("");
            float map_y = (*it).arTask[i].command.map_y;
            stringStream << "map_y= " << twodec(map_y);
            string map_y_str = stringStream.str();
            painter.drawText(x_offset,y_offset + 60 + nBlockHeight*i + 20* 3, QString::fromStdString(map_y_str));
            stringStream.str("");
            float map_yaw = (*it).arTask[i].command.map_yaw;
            stringStream << "map_yaw= " << twodec(map_yaw);
            string map_yaw_str = stringStream.str();
            painter.drawText(x_offset,y_offset + 60 + nBlockHeight*i + 20* 4, QString::fromStdString(map_yaw_str));
        }
        nMCount ++;
	}

    // [二] 机器人状态列表
    int state_display_y = height() - 270;
    penTitle.setColor(Qt::white);
	painter.setPen(penTitle);
    painter.drawLine(QPointF(0, state_display_y-30), QPointF(width(), state_display_y-30));
    // cv::line(ui_image,Point(10,state_display_y-30),Point(UI_WIDTH-10,state_display_y-30),cv::Scalar(255,255,255),1);
    // 标题
    penTitle.setColor(Qt::yellow);
	painter.setPen(penTitle);
    fontText.setPointSize(18);
    painter.setFont(fontText);
    strMissionTitle = QString("机器人状态列表");
    painter.drawText(5, state_display_y, strMissionTitle);

    int r_count = 0;
    int r_width = 250;
    int r_height = 250;
    QRect rcRobot(0,0,240,r_height);
    QRect rcStatus(0,0,230,r_height-55);
    QBrush brStatus;
	brStatus.setStyle(Qt::SolidPattern);
    fontText.setPointSize(12);
    painter.setFont(fontText);
    // 遍历机器人状态数组
    for(int i=0;i<ROBOT_NUM;i++)
    {
        int x_offset = 15 + r_count * r_width;
        int y_offset = state_display_y + 10;

        rcRobot.moveTo(x_offset-10, y_offset);
        rcStatus.moveTo(x_offset-5, y_offset+48);
        // 画机器人状态矩形背景
        if(pMisMgr->robot_list[i].state == T_ST_IN_PROGRESS)
        {
            brBg.setColor(QColor(0, 155, 0));
            brStatus.setColor(QColor(50, 0, 50));
            penText.setColor(QColor(255, 255, 255));
        } else if(pMisMgr->robot_list[i].state == T_ST_OFFLINE)
        {
            brBg.setColor(QColor(100, 100, 100));
            brStatus.setColor(QColor(70, 70, 70));
            penText.setColor(QColor(150, 150, 150));
        }else
        {
            brBg.setColor(QColor(100, 100, 00));
            brStatus.setColor(QColor(50, 0, 50));
            penText.setColor(QColor(255, 255, 255));
        }
	    painter.setBrush(brBg);
        painter.drawRect(rcRobot);	
	    painter.setBrush(brStatus);
        painter.drawRect(rcStatus);
	    painter.setPen(penText);

        // [3-1] id
        stringStream.str("");
        int robot_id = pMisMgr->robot_list[i].info.id;
        stringStream << "机器人ID = " << robot_id;
        string id_str = stringStream.str();
        painter.drawText(x_offset,y_offset+20, QString::fromStdString(id_str));
        // [3-2] dev 设备类型
        stringStream.str("");
        int dev_type = pMisMgr->robot_list[i].info.dev_type;
        stringStream << "设备类型 " << str_helper.GetDevTypeStr(dev_type);
        string dev_str = stringStream.str();
        painter.drawText(x_offset,y_offset+20 + 20, QString::fromStdString(dev_str));
        // [3-3] 分配的任务指令 
        stringStream.str("");
        int cmd_recv = pMisMgr->robot_list[i].info.cmd_recv;
        stringStream << "接到指令 " << str_helper.GetCommandStr(cmd_recv);
        string cmd_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*2, QString::fromStdString(cmd_str));
        // [3-4] 工作状态 
        stringStream.str("");
        int task_state = pMisMgr->robot_list[i].state;
        stringStream << "当前任务 " << str_helper.GetTaskStateStr(task_state);
        string task_state_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*3, QString::fromStdString(task_state_str));
        // [3-5] 行为状态
        stringStream.str("");
        int info_state = pMisMgr->robot_list[i].info.state;
        stringStream << "当前状态 " << str_helper.GetRobotStateStr(info_state);
        string info_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*4, QString::fromStdString(info_str));
        // [3-6] map_x
        stringStream.str("");
        float map_x = pMisMgr->robot_list[i].info.map_x;
        stringStream << "map_x= " << twodec(map_x);
        string map_x_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*5, QString::fromStdString(map_x_str));
        // [3-7] map_y
        stringStream.str("");
        float map_y = pMisMgr->robot_list[i].info.map_y;
        stringStream << "map_y= " << twodec(map_y);
        string map_y_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*6, QString::fromStdString(map_y_str));
        // [3-8] map_yaw
        stringStream.str("");
        float map_yaw = pMisMgr->robot_list[i].info.map_yaw;
        stringStream << "map_yaw= " << twodec(map_yaw);
        string map_yaw_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*7, QString::fromStdString(map_yaw_str));
        // [3-9] 电池电量
        stringStream.str("");
        float battery = pMisMgr->robot_list[i].info.battery;
        stringStream << "电量= " << twodec(battery) << " V";
        string battery_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*8, QString::fromStdString(battery_str));
        // [3-10] 料盒颜色
        stringStream.str("");
        int box_color = pMisMgr->robot_list[i].info.box_color;
        stringStream << "box_clr= " << box_color;
        string box_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*9, QString::fromStdString(box_str));
        // [3-10] 接收到的数据包 
        stringStream.str("");
        int package_recv = pMisMgr->robot_list[i].package_recv;
        int package_recv_last = pMisMgr->robot_list[i].package_recv_last;
        stringStream << "网络数据包 "<<package_recv_last << " / " << package_recv;
        string pack_recv_str = stringStream.str();
        painter.drawText(x_offset,y_offset+30 + 20*10, QString::fromStdString(pack_recv_str));
    
        r_count ++;
    }
}