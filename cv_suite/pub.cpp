#include "pch.h"
#include "pub.h"
#include <zmq.h>
#include <cstdio>
namespace publisher {
    TrajectoryPublisher::TrajectoryPublisher(unsigned int port) :
        context(nullptr),
        publisher(nullptr) {
        context = zmq_ctx_new();
        publisher = zmq_socket(context, ZMQ_PUB);
        char addr_buf[100];
        std::snprintf(addr_buf, sizeof(addr_buf), "tcp://*:%u", port);
        zmq_bind(publisher, addr_buf);
    }

    TrajectoryPublisher::~TrajectoryPublisher() {
        zmq_close(publisher);
        zmq_ctx_destroy(context);
    }

    bool TrajectoryPublisher::send_points(Trajectory now_point) {
        auto sz = zmq_send(publisher, &now_point, sizeof(now_point), 0);
        return sz == sizeof(now_point);
    }
    bool TrajectoryPublisher::send_points(double x, double y, double z, double w, double p, double r) {
        Trajectory trj{ x, y, z, w, p, r };
        return send_points(trj);
    }
    RobotStatePublisher::RobotStatePublisher(unsigned int port) :
        context(nullptr),
        publisher(nullptr) {
        context = zmq_ctx_new();
        publisher = zmq_socket(context, ZMQ_PUB);
        char addr_buf[100];
        std::snprintf(addr_buf, sizeof(addr_buf), "tcp://*:%u", port);
        zmq_bind(publisher, addr_buf);
    }
    RobotStatePublisher::~RobotStatePublisher() {
    }
    bool RobotStatePublisher::send_points(Trajectory now_point) {
        return false;
    }
}