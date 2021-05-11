#include "Main.hpp"

int coffset [] = {
    0,
    128,
    142,
    135,
    113,
    94,
    114,
    152,
    151,
    158
};

cv::Mat findAngles(cv::Mat image, float binning){
    cv::Mat sobelx, sobely, magnitudes, angles;

    cv::Sobel(image, sobelx, CV_32F, 1, 0, 1);
    cv::Sobel(image, sobely, CV_32F, 0, 1, 1);
    cv::cartToPolar(sobelx, sobely, magnitudes, angles, true);

    cv::resize(angles, angles, cv::Size(), binning, binning, cv::INTER_AREA );

    return angles;
}

cv::Point getPosition(cv::Mat &search){
    double val_min, val_max;
    cv::Point loc_min, loc_max;
    cv::minMaxLoc(search, &val_min, &val_max, &loc_min, &loc_max);
    return loc_max;
}

bool confidence(Images *images, cv::Mat&result){
    cv::Mat heat_graph;
    cv::matchTemplate(images->current_image, images->pattern_image, heat_graph, cv::TM_CCOEFF_NORMED);
    float similarity = heat_graph.at<float>(0,0);

    int rows = result.rows;
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);
    cv::reduce(result, result, 0, cv::REDUCE_SUM);
    float noise = result.at<float>(0,0)/rows;
    noise = 1/(noise*10);

    float similarity_weight = 0.5;
    float noise_weight = 1;

    float confidence_value = (similarity*similarity_weight)+(noise*noise_weight);

    return (confidence_value>0.61);
}

void computeSpeed(Images *images){
    cv::Mat image, templat, heat_map;
    float scale_factor = 0.5;
    int margin_y = 60;

    cv::Rect i_roi(0, margin_y,images->current_image.cols,images->current_image.rows-(margin_y*2));
    cv::Rect p_roi(6,6,4,130);
    
    cv::resize( images->current_image(i_roi), image, cv::Size(), scale_factor, scale_factor );
    cv::resize( (images->previous_image(i_roi))(p_roi), templat, cv::Size(), scale_factor, scale_factor );
    
    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    
    cv::Point position = getPosition(heat_map);

    int travel = (position.x/scale_factor) - 6;
    images->travel = travel*4;
}

void computeMovement(Images *images){
    computeSpeed(images);

    cv::Mat angles, result, result1, result2;

    angles = findAngles(images->current_image);

    int center_cam = coffset[images->module_number]/2;

    if (images->program == 1){
        cv::Rect roi( 0, (center_cam)-20, images->pattern_angles.cols, 40);
        cv::matchTemplate(angles, images->pattern_angles(roi), result, cv::TM_CCOEFF_NORMED);
        //std::cout << "Tufted" << std::endl;
    }
    else{
        cv::matchTemplate(angles, images->synthetic_template, result1, cv::TM_CCOEFF_NORMED);
        cv::matchTemplate(angles, images->synthetic_template_interted, result2, cv::TM_CCOEFF_NORMED);
        cv::absdiff(result1, result2, result);
        //std::cout << "printed/v202" << std::endl;
    }

    cv::Point position = getPosition(result);

    int shift = (position.y+8)*2;

    if (confidence(images,result))
        images->shift = -((shift-(center_cam*2))*4);
    else
        images->shift = 0;
}

bool loaded = false;
void getMovement(Images *local_set){
    std::chrono::time_point<std::chrono::system_clock> start_comp, end_comp;

    std::chrono::milliseconds dF = std::chrono::duration_cast<std::chrono::milliseconds>(local_set->c_stamp-local_set->p_stamp);
    local_set->frame_gap = dF.count();

    start_comp = std::chrono::system_clock::now();
    ///////////////////////////////////////////////////////////////////////////////////////
    if (loaded && dF.count() <=300){
        computeMovement(local_set);
    }
    local_set->previous_image = local_set->current_image.clone();
    local_set->p_stamp = local_set->c_stamp;
    loaded = true;
    ///////////////////////////////////////////////////////////////////////////////////////
    end_comp = std::chrono::system_clock::now();

    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);
    //std::cout << " dF: " << dF.count() << " dC: " << dV.count() << " " << std::endl;
}