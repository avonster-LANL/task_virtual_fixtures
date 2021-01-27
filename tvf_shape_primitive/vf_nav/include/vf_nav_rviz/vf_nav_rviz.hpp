//  The MIT License (MIT)
//
//  Copyright (c) 2015 Andrew Sharp
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.


#ifndef VF_NAV_RVIZ_HPP
#define VF_NAV_RVIZ_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <actionlib/client/simple_action_client.h>
#include <vf_nav/VFNavAction.h>
#include <vf_nav/VFNavGoal.h>
#endif

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QLabel>

#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class QTabWidget;

namespace vf_nav
{

class VFNavTab : public QWidget
{
    Q_OBJECT

public:
    explicit VFNavTab(bool isPathTab, QWidget *parent = 0);

    QVBoxLayout* layout;
    QHBoxLayout *xButtonLayout;
    QHBoxLayout *yButtonLayout;
    QHBoxLayout *zButtonLayout;
    QHBoxLayout *addRemoveLayout;
    QHBoxLayout *planExecuteLayout;
    QHBoxLayout *clearLayout;

    QPushButton* xPosButton_;
    QPushButton* xNegButton_;
    QPushButton* yPosButton_;
    QPushButton* yNegButton_;
    QPushButton* zPosButton_;
    QPushButton* zNegButton_;
    QPushButton* addPtButton_;
    QPushButton* delPtButton_;
    QPushButton* planPtsButton_;
    QPushButton* executePtsButton_;
    QPushButton* clearPtsButton_;
};

class VFNavPanel: public rviz::Panel
{
  Q_OBJECT  // necessary macro
public:
  VFNavPanel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:
void xPos();
void xNeg();
void yPos();
void yNeg();
void zPos();
void zNeg();
void addPt();
void delPt();
void planPts();
void executePts();
void clearPts();

private:  // member variables
// ROS node handle
ros::NodeHandle nh_;

// action client to send motion commands
actionlib::SimpleActionClient<vf_nav::VFNavAction> ac_;

// goal for motion commands
vf_nav::VFNavGoal goal_;

QTabWidget *tabWidget_;

VFNavTab *poseTab;
VFNavTab *pathTab;

int PoseIndex;
int PathIndex;

};

} // end namespace vf_nav

#endif // MOTION_COMMAND_RVIZ_HPP
