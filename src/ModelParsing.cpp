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

#include "ModelParsing.h"
#include <iostream>
#include <yarp/os/ResourceFinder.h>
#include "iCub/iDynTree/iCubTree.h"

namespace codyco {

    bool iCubURDFModelFromRf(yarp::os::ResourceFinder & rf,
                        std::string & icub_urdf)
    {
        // First try to load urdf from local option
        if( rf.check("urdf") )
        {
            std::string urdf_file = rf.find("urdf").asString().c_str();
            std::string icub_urdf = rf.findFile(urdf_file.c_str());
            return true;
        }

        // Otherwise try to load it from the wbi configuration files
        yarp::os::Property yarpWbiOptions;
        //Get wbi options from the canonical file
        if( !rf.check("wbi_conf_file") )
        {
            std::cerr << "[ERR] impossible to open wholeBodyInterface: wbi_conf_file option missing" << std::endl;
        }
        std::string wbiConfFile = rf.findFile("wbi_conf_file");
        yarpWbiOptions.fromConfigFile(wbiConfFile);

        if( !yarpWbiOptions.check("urdf_file") )
        {
            std::cerr << "[ERR] urdf_file not found in configuration files" << std::endl;
            return false;
        }

        std::string urdf_file = yarpWbiOptions.find("urdf_file").asString().c_str();
        icub_urdf = rf.findFile(urdf_file.c_str());

        return true;
    }


}
