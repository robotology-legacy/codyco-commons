/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "ControllerThread.h"

namespace codyco {

    ControllerThreadDelegate::~ControllerThreadDelegate() {}

    ControllerThread::ControllerThread(int period) : yarp::os::RateThread(period) {}

    ControllerThread::~ControllerThread() {}

    void ControllerThread::setActiveState(bool isActive) {}

    bool ControllerThread::isActiveState() {return false;}

    void ControllerThread::setDelegate(ControllerThreadDelegate* delegate) {}

    ControllerThreadDelegate* ControllerThread::delegate() {return 0;}

    void ControllerThread::attachDebugVariable(double *buffer, int size) {}
    void ControllerThread::attachDebugVariable(Eigen::MatrixXd variable, std::string portName) {}
    void ControllerThread::detachDebugVariable() {}

#pragma mark - RateThread methods
    bool ControllerThread::threadInit() {return false;}
    void ControllerThread::threadRelease() {}
    void ControllerThread::run() {}

    void ControllerThread::controllerWillActivate() {}
    void ControllerThread::controllerWillDeactivate() {}

    void ControllerThread::controllerWillRunLoop() {}
    void ControllerThread::controllerDidRunLoop() {}
    
}
