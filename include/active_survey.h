#pragma once

#include "active_survey_param.h"

#include "Eigen/Core"
#include "ros/ros.h"

using namespace Eigen;

namespace asn
{

#define active_survey_log( ... ) if(active_survey_param::logging){printf("logging ..... \n");fprintf(active_survey::log_file_,"%f ", ros::Time::now().toSec());fprintf(active_survey::log_file_, __VA_ARGS__ );fflush(active_survey::log_file_);printf("logging ..... finished \n");}
#define save_log() if(active_survey_param::logging){fclose(active_survey::log_file_);active_survey::log_file_ = fopen(active_survey::log_file_name_.c_str(), "a+");}


class active_survey
{
public:

    ~active_survey();

    static active_survey * instance(int argc=0, char **argv=NULL)
    {
        if(instance_ == NULL)
        {
            instance_ = new active_survey(argc, argv);
        }

        return instance_;
    }

    void update();
    void draw();
    void hanlde_key_pressed(std::map<unsigned char, bool> &key, bool &update_key);

    static FILE* log_file_;

private:
    active_survey(int argc, char **argv);
    void setup_log_file();
    Vector3f get_color(double h);

    static active_survey* instance_;
    ros::NodeHandle nh_;
    static std::string log_file_name_;
};

}
