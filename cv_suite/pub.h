#pragma once
namespace publisher
{
    // Continuous 6 doubles.
    struct Trajectory
    {
        double x;
        double y;
        double z;
        double w;
        double p;
        double r;
        Trajectory(double x_, double y_, double z_, double w_, double p_, double r_) :
            x(x_), y(y_), z(z_), w(w_), p(p_), r(r_) {}
    };

    class TrajectoryPublisher {
        void* context;
        void* publisher;
    public:
        TrajectoryPublisher(unsigned int port);
        ~TrajectoryPublisher();
        bool send_points(Trajectory now_point);
        bool send_points(double x, double y, double z, double w, double p, double r);
    };

    class RobotStatePublisher {
        void* context;
        void* publisher;
    public:
        RobotStatePublisher(unsigned int port);
        ~RobotStatePublisher();
        bool send_points(Trajectory now_point);
    };
}
