#include "pch.h"

#include "test.h"
namespace common {
    namespace test {
        mytimer::mytimer(std::ostream& os)
            // os is hint; implementation may ignore, particularly in embedded systems
            : boost::timer(), m_tag(), m_os(os) {}
        mytimer::mytimer(std::string tag, std::ostream& os)
            : boost::timer(), m_tag(tag), m_os(os) {}
        mytimer::~mytimer()
        {
            //  A) Throwing an exception from a destructor is a Bad Thing.
            //  B) The progress_timer destructor does output which may throw.
            //  C) A progress_timer is usually not critical to the application.
            //  Therefore, wrap the I/O in a try block, catch and ignore all exceptions.
            try
            {
                // use istream instead of ios_base to workaround GNU problem (Greg Chicares)
                std::istream::fmtflags old_flags = m_os.setf(std::istream::fixed,
                    std::istream::floatfield);
                std::streamsize old_prec = m_os.precision(2);
                m_os << m_tag << " " << elapsed() << " s\n" // "s" is System International d'Unites std
                    << std::endl;
                m_os.flags(old_flags);
                m_os.precision(old_prec);
            }

            catch (...) {} // eat any exceptions
        }
    }
}