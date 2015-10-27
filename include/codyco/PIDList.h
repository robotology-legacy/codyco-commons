#ifndef CODYCOLIB_PIDLIST_H
#define CODYCOLIB_PIDLIST_H

#include <string>
#include <cstdlib>

namespace yarp {
    namespace dev {
        class Pid;
    }
}

namespace codyco {
    class PIDList;
}

class codyco::PIDList {
private:
    size_t m_size;
    yarp::dev::Pid *m_pidList;
public:

    explicit PIDList(size_t _size);

    PIDList(const PIDList& list);

    ~PIDList();

    PIDList& operator=(const PIDList& list);

    size_t size() const;
    yarp::dev::Pid * const pidList() const;

    std::string description() const;

    friend std::ostream& operator<<(std::ostream& stream, const PIDList& list);
};

#endif /* end of include guard: CODYCOLIB_PIDLIST_H */
