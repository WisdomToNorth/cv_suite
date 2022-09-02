#ifndef CVSUITE_SOURCE_DRIVER_TEST_H
#define CVSUITE_SOURCE_DRIVER_TEST_H
#include <iostream>
#include <string>

#include <boost/timer.hpp>

namespace common {
    namespace test {
        class mytimer : public boost::timer {

        public:
            explicit mytimer(std::ostream& os = std::cout);
            mytimer(std::string tag, std::ostream& os = std::cout);
            ~mytimer();
            //Noncopyable
            mytimer(const mytimer&) = delete;
            mytimer& operator=(const mytimer&) = delete;
        private:
            std::string m_tag;
            std::ostream& m_os;
        };
    }
}
#endif