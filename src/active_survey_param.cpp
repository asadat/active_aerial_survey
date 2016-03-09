#include "active_survey_param.h"

namespace asn
{

double active_survey_param::area_rotation = 0;
double active_survey_param::cx = 0;
double active_survey_param::cy = 0;
double active_survey_param::sensingTime = 2;
double active_survey_param::min_footprint = 4.0;
std::string active_survey_param::log_folder = std::string("");
bool active_survey_param::logging = false;
double active_survey_param::FOV = 3.14/2.0;
std::string active_survey_param::interesting_label = "grass";
std::string active_survey_param::training_set_dir = "";
int active_survey_param::image_w = 640;
int active_survey_param::image_h = 480;
bool active_survey_param::simulation = true;
bool active_survey_param::interesting_simulation = false;
bool active_survey_param::visualization = true;
int active_survey_param::bf_sqrt = 2;
double active_survey_param::speed = 1.0;
double active_survey_param::area_width = 50;
double active_survey_param::area_height = 50;
bool active_survey_param::sim_running = true;
double active_survey_param::percent_interesting = 20;
int active_survey_param::patches = 5;
bool active_survey_param::auto_exit = true;
std::string active_survey_param::strategy = "lm";
bool active_survey_param::bypass_controller = false;
double active_survey_param::int_prob_thr = 0.31;
double active_survey_param::alpha_h0 = 0.1;
double active_survey_param::alpha_hm = 0.4;
double active_survey_param::beta_h0 = 0.1;
double active_survey_param::beta_hm = 0.4;
double active_survey_param::prob_r = 0.8;
double active_survey_param::pathCost = 100;
std::string active_survey_param::prior_file_name = "prior.jpg";
std::string active_survey_param::nuc_dir = "";
int active_survey_param::high_res_cells = 3;
std::string active_survey_param::policy = "dfs";
double active_survey_param::average_speed = 2.0;
double active_survey_param::turning_time = 1.0;
double active_survey_param::time_limit = 600.0;
int active_survey_param::lm_tracks = 4;
double active_survey_param::random_seed = -1;
double active_survey_param::gp_sigma = 0.1;
double active_survey_param::exploitation_rate = 0.1;
double active_survey_param::coarse_coverage_height = 10.0;
double active_survey_param::sensing_height = 2.0;
double active_survey_param::uncertain_cells_threshold = 10.0;
double active_survey_param::discount_factor = 0.999;

int active_survey_param::non_ros::cell_drawing_mode = 0;
double active_survey_param::non_ros::beta = 0.4;
double active_survey_param::non_ros::target_threshold = 0.25;

int active_survey_param::run_number = 0;

void active_survey_param::GetParams(ros::NodeHandle &nh)
{
    nh.param<double>("FOV",FOV,1.57);
    nh.param<double>("area_rotation",area_rotation,0);
    nh.param<double>("sensing_time",sensingTime,2);
    nh.param<double>("min_footprint",min_footprint,2);
    nh.param("logging", logging, false);
    nh.param("log_folder", log_folder, std::string(""));
    nh.param("interesting_label", interesting_label, std::string(""));
    nh.param("training_set_dir", training_set_dir, std::string(""));
    nh.param<int>("image_w",image_w,640);
    nh.param<int>("image_h",image_h,480);
    nh.param("strategy", strategy, std::string("lm"));

    nh.param<bool>("simulation", simulation, true);
    nh.param<bool>("interesting_simulation", interesting_simulation, true);
    nh.param<bool>("visualization", visualization, true);
    nh.param<int>("branching_sqrt",bf_sqrt,2);
    nh.param<double>("speed",speed,1.0);
    nh.param<double>("area_width",area_width,50);
    nh.param<double>("area_height",area_height,50);
    nh.param<int>("patches",patches,5);
    nh.param<double>("percent_interesting",percent_interesting,30.0);
    nh.param<bool>("auto_exit", auto_exit, true);
    nh.param<bool>("bypass_controller", bypass_controller, false);

    nh.param<double>("int_prob_thr",int_prob_thr,0.31);
    nh.param<double>("alpha_h0",alpha_h0,0.1);
    nh.param<double>("alpha_hm",alpha_hm,0.4);
    nh.param<double>("beta_h0",beta_h0,0.1);
    nh.param<double>("beta_hm",beta_hm,0.4);
    nh.param<double>("prob_r",prob_r,0.8);

    nh.param<double>("path_cost",pathCost,100.0);

    nh.param("prior_file_name", prior_file_name, std::string("prior.jpg"));
    nh.param("nuc_dir", nuc_dir, std::string(""));


    nh.param("high_res_cells", high_res_cells, 3);
    nh.param("policy", policy, std::string("dfs"));
    nh.param<double>("time_limit",time_limit,600.0);
    nh.param<double>("average_speed",average_speed,2.0);
    nh.param<double>("turning_time",turning_time,1.0);
    nh.param<int>("lm_tracks", lm_tracks, 4);
    nh.param<double>("random_seed", random_seed, -1.0);
    nh.param<double>("gp_sigma", gp_sigma, 0.1);
    nh.param<double>("exploitation_rate", exploitation_rate, 0.1);

    nh.param<double>("coarse_coverage_height", coarse_coverage_height, 10.0);
    nh.param<double>("sensing_height", sensing_height, 2.0);
    nh.param<double>("uncertain_cell_threshold", uncertain_cells_threshold, 10.0);
    nh.param<double>("discount_factor", discount_factor, 0.999);

    nh.param<int>("run_number", run_number, 0);


}

}
