#include <iostream>
#include "active_survey.h"
#include "GL/glut.h"
#include "environment_model/environment_model.h"

using namespace Eigen;

namespace asn
{

FILE *active_survey::log_file_ = NULL;
std::string active_survey::log_file_name_ = std::string("");
std::shared_ptr<active_survey> active_survey::instance_;

active_survey::active_survey(int argc, char **argv):
    nh_("active_survey")
{    
    ros::NodeHandle private_node_handle_("~");
    active_survey_param::GetParams(private_node_handle_);


    mav_ = std::make_shared<mav>(*environment_model::instance(), Vector3f{-0.5f*static_cast<float>(active_survey_param::area_width),
                                                                         -0.5f*static_cast<float>(active_survey_param::area_height),
                                                                         10.0f});

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

    gaussian_field::reset();
    environment_model::reset();
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
    mav_->draw();

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


bool active_survey::update()
{    
//    static std::clock_t last_clk = std::clock();

//    //printf("%.5f\n", (std::clock()-last_clk)/((double)CLOCKS_PER_SEC));
//    last_clk = std::clock();

    static double ros_freq=15.0;
    static double ros_period = 1/ros_freq;
    static ros::Time last_time = ros::Time::now();
    static ros::Time last_time_mav = ros::Time::now();

    ros::Time cur_time = ros::Time::now();

    double dt = (cur_time-last_time).toSec();
    if( dt > ros_period)
    {
        last_time = cur_time;
        if(ros::ok())
        {
            ros::spinOnce();          
        }
        else
        {
            ROS_ERROR("ROS not OK !!!!!!!!!!");
            exit(0);
        }
    }

    double dt_mav = (cur_time-last_time_mav).toSec();
    if(dt_mav > 0.001)
    {
        last_time_mav = cur_time;

        // if planning took too long stop
        // the timer
        if(dt_mav > 0.1)
            dt_mav = 0.1;

        ROS_INFO("mav_update start ....");
        bool  mav_res = mav_->update(dt_mav);
        ROS_INFO("mav_update end %d ....", mav_res);

        return  mav_res;
    }

    return true;
}

void active_survey::hanlde_key_pressed(std::map<unsigned char, bool> &key, bool &updateKey)
{    
    if(key['1'])
    {
        environment_model::instance()->generate_environment(true);
        updateKey = false;
    }
    else if(key['2'])
    {
        active_survey_param::non_ros::cell_drawing_mode+=1;
        active_survey_param::non_ros::cell_drawing_mode %= 7;
        updateKey = false;
    }
    else if(key[']'])
    {
        mav_->set_goal({utility::random_number_f(-0.5*active_survey_param::area_width,  0.5*active_survey_param::area_width),
                        utility::random_number_f(-0.5*active_survey_param::area_height, 0.5*active_survey_param::area_height), 10});
        updateKey = false;
    }
    else if(key['['])
    {
        mav_->stop();
        updateKey = false;
    }
    else if(key['p'])
    {
        mav_->sense();
        updateKey = false;
    }
    else if(key['='])
    {
        active_survey_param::speed += 0.5;
        updateKey = false;
    }
    else if(key['-'])
    {
        active_survey_param::speed -= 0.5;
        updateKey = false;
    }
    else if(key['o'])
    {
        mav_->stop();
    }
    else if(key['i'])
    {
        mav_->resume();
    }
    else if(key['u'])
    {
        for(auto cit=environment_model::instance()->get_grid()->begin();
            cit!=environment_model::instance()->get_grid()->end(); ++cit)
        {
            auto &cell = *cit;
            double x[] = {cell->get_center()[0],cell->get_center()[1]};
            cell->set_estimated_value(gaussian_field::instance()->f(x));
            cell->set_variance(gaussian_field::instance()->var(x));
        }

    }


//    else if(key['w'])
//    {
//        mav_->change_velocity({0,0.01,0});
//    }
//    else if(key['s'])
//    {
//        mav_->change_velocity({0,-0.01,0});
//    }
//    else if(key['d'])
//    {
//        mav_->change_velocity({0.01,0,0});
//    }
//    else if(key['a'])
//    {
//        mav_->change_velocity({-0.01,0,0});
//    }

}

}
