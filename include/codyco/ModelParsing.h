/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
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

#ifndef CODYCOLIB_MODELPARSING_H
#define CODYCOLIB_MODELPARSING_H

#include <string>

namespace yarp {
    namespace os {
        class ResourceFinder;
    }
}


namespace codyco
{
    bool iCubURDFModelFromRf(yarp::os::ResourceFinder & rf,
                             std::string & icub_urdf);
}


#endif
