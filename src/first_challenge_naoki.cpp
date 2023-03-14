#include "first_challenge_naoki/first_challenge_naoki.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    odom_sub_ = nh_.subscribe("/roomba/odometry", 1, &FirstChallenge::odometry_callback, this);
    laser_sub_ = nh_.subscribe("/scan", 1, &FirstChallenge::laser_callback, this);
    cmd_vel_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::run(float v, float r)
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = v;
    cmd_vel_.cntl.angular.z = r;

    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::straght()
{
    run(0.20, 0.0);
}

void FirstChallenge::turn()
{
    run(0.0, 0.30);
}

void FirstChallenge::stop()
{
    run(0.0, 0.0);
}

float FirstChallenge::scan()
{
    float range_min = 1e6;
    if(laser_.ranges.size()){
        int sight = 10.0 / 0.125;
        int laser_front = laser_.ranges.size() / 2;
        for(int i = laser_front - sight ; i < laser_front + sight; i++){
            if(laser_.ranges[i] < range_min && laser_.ranges[i] > 0.20){
                range_min = laser_.ranges[i];
            }
        }
    }
    return range_min;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    init_theta = tf2::getYaw(odometry_.pose.pose.orientation);
    turning1_ = false;
    int count = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        current_theta = tf2::getYaw(odometry_.pose.pose.orientation);

        if(scan() >= 0.50)
        {
           /* printf("init = %lf\n", init_theta);
            printf("current = %lf\n", current_theta);
            printf("x = %lf\n", odometry_.pose.pose.position.x);*/

            if(odometry_.pose.pose.position.x <= 1.0){
                straght();
               /* printf("straght1");*/
            }
            else if (!turning1_){
                if(fabs(init_theta - current_theta) <= 0.1){
                    count++;
                   /* printf("count = %d\n", count);*/
                }
                if(count >= 0){
                    turning1_ = true;
                }

                turn();
                /*printf("turn");*/
            }
            else{
                straght();
                /*printf("straght2");*/
            }
        }
        else
        {
            stop();
            /*printf("stop");*/
        }
        loop_rate.sleep();
    }
}

/*
void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        run();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge_naoki");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
