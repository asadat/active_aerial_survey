
#include "active_survey.h"
#include "GL/glut.h"
#include "environment_model.h"

using namespace Eigen;

namespace asn
{

FILE *active_survey::log_file_ = NULL;
std::string active_survey::log_file_name_ = std::string("");
std::shared_ptr<active_survey> active_survey::instance_;

active_survey::active_survey(int argc, char **argv):nh_("active_survey")
{    
    ros::NodeHandle private_node_handle_("~");
    active_survey_param::GetParams(private_node_handle_);

    if(active_survey_param::logging)
    {
        setup_log_file();
    }

    active_survey_log("this is a test for active_survey logging system.");
}

active_survey::~active_survey()
{
    if(active_survey_param::logging)
    {
        fclose(log_file_);
    }
}

void active_survey::setup_log_file()
{
    time_t now = time(0);
    char* dt = ctime(&now);
    log_file_name_ = std::string(dt);
    for(size_t i=0; i<log_file_name_.length();i++)
    {
        printf("%d ",log_file_name_[i]);
        if(log_file_name_[i] == ' ' || log_file_name_[i] == ':' || log_file_name_[i] == 10)
            log_file_name_[i] = '_';
    }

    log_file_name_ = active_survey_param::log_folder+"/"+log_file_name_+".log";

    ROS_INFO("log file path: %s", log_file_name_.c_str());
    log_file_ = fopen(log_file_name_.c_str(), "w");
}

void active_survey::draw()
{       
    environment_model::instance()->draw();

//    glLineWidth(3);
//    glColor3f(1,0,0);
//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    glBegin(GL_QUADS);

//    utility::gl_vertex2f({-14.5, -4.5});
//    utility::gl_vertex2f({ -14.5  , 14});
//    utility::gl_vertex2f({ 41  , 14});
//    utility::gl_vertex2f({41, -4.5});
//    glEnd();
}


void active_survey::update()
{    
    static double ros_freq=15.0;
    static double ros_period = 1/ros_freq;
    static ros::Time last_time = ros::Time::now();

    double dt = (ros::Time::now()-last_time).toSec();
    if( dt > ros_period)
    {
        last_time = ros::Time::now();
        if(ros::ok())
        {
            ros::spinOnce();          
        }
        else
        {
            exit(0);
        }
    }

//    double dtmav = (ros::Time::now()-lastTimeMav).toSec();
//    if(active_survey_param::bypass_controller || dtmav > 0.001)
//    {
//        lastTimeMav = ros::Time::now();
//        if(!active_survey_param::bypass_controller)
//        {
//            mav.Update(dtmav);
//            if((lastTimeMav-lastSensing).toSec() > 2)
//            {
//                lastSensing = lastTimeMav;
//                //traversal->SensingUpdate(mav.GetPos());
//            }
//        }

//        if(active_survey_param::bypass_controller || mav.AtGoal())
//        {
//            OnReachedGoal();
//        }
//    }
}

void active_survey::hanlde_key_pressed(std::map<unsigned char, bool> &key, bool &updateKey)
{    
    if(key['1'])
    {
        environment_model::instance()->generate_environment();
        updateKey = false;
    }
    else if(key['2'])
    {
        active_survey_param::non_ros::cell_drawing_mode+=1;
        active_survey_param::non_ros::cell_drawing_mode %= 4;
        updateKey = false;
    }
}

}
