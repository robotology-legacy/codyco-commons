#include "PIDList.h"

#include <yarp/dev/ControlBoardPid.h>
#include <sstream>

namespace codyco {

    PIDList::PIDList(size_t _size): m_size(_size), m_pidList(0)
    {
        m_pidList = new yarp::dev::Pid[m_size];
    }

    PIDList::PIDList(const PIDList& list): m_size(list.m_size), m_pidList(0) {
        m_pidList = new yarp::dev::Pid[m_size];
        for (int i = 0; i < m_size; ++i) {
            m_pidList[i] = list.m_pidList[i];
        }
    }

    PIDList::~PIDList() {
        if (m_pidList) {
            delete [] m_pidList;
            m_pidList = 0;
        }
        m_size = 0;
    }

    PIDList& PIDList::operator=(const PIDList& list) {
        if (this == &list) return *this;

        if (this->m_pidList) {
            delete [] m_pidList;
        }
        this->m_size = list.m_size;
        m_pidList = new yarp::dev::Pid[m_size];
        for (int i = 0; i < m_size; ++i) {
            m_pidList[i] = list.m_pidList[i];
        }
        return *this;
    }

    std::string PIDList::description() const {
        std::ostringstream stream;
        for (int i = 0; i < m_size; ++i) {
            stream << "kp: " << m_pidList[i].kp << " kd: " << m_pidList[i].kd << " ki " << m_pidList[i].ki << "\n";
        }
        return stream.str();
    }

    size_t PIDList::size() const
    {
        return m_size;
    }

    yarp::dev::Pid * const PIDList::pidList() const
    {
        return m_pidList;
    }

    std::ostream& operator<<(std::ostream& stream, const PIDList& list)
    {
        return stream << list.description();
    }

}