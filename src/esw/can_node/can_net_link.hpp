#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

namespace mrover {

    struct CanNetLink {

        CanNetLink();

        ~CanNetLink();

        nl_sock* mSocket;
        rtnl_link* mLink;
    };

} // namespace mrover
