#ifndef MISSION_INFO_WIDGET_H
#define MISSION_INFO_WIDGET_H

#include <QtGui>
#include <QApplication>
#include <QtWidgets/QWidget>
#include <QScrollArea>
#include <list>
#include <sstream>
#include <stdlib.h>
#include "Struct.h"
#include "Common.h"
#include "StringHelper.h"
#include "MissionManager.h"

class CMissionInfoWidget : public QWidget
{
    Q_OBJECT
public:
    CMissionInfoWidget(QWidget *parent = Q_NULLPTR);
    ~CMissionInfoWidget();
    void paintEvent(QPaintEvent *event);

    CMissionManager* pMisMgr;
    CStringHelper str_helper;
// public slots:

// signals:
    
// protected:

// private:
    
};

#endif // MISSION_INFO_WIDGET_H