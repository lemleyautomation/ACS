#include "Main.hpp"

cv::Mat image, templat, heat_map;

void computeMovement(Images *images){
    
    float scale_factor = 0.5;
    int width = 320;
    int height = 156;
    int m_x = 6;
    int m_y = 8;
    int d_x = 4;
    int d_y = 140;

    cv::Rect roi(m_x,m_y,d_x,d_y);
    
    cv::resize( images->current_image,
                image,
                cv::Size(),
                scale_factor, scale_factor );
    
    cv::resize( images->previous_image(roi),
                templat,
                cv::Size(),
                scale_factor, scale_factor );

    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;
    
    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    
    int travel = (loc_max.x/scale_factor) - m_x;
    images->travel = travel*4;

    cv::Rect e_roi(0,58,320,40);
    
    cv::resize( images->current_image,
                image,
                cv::Size(),
                scale_factor, 1 );
    
    cv::resize( images->pattern_image(e_roi),
                templat,
                cv::Size(),
                scale_factor, 1 );
    
    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    int shift = loc_max.y - 58;

    images->shift = shift * 4;
    
}

bool computeMovement2(Images *images){

    float scale_factor = 0.5;
    int m_x = 6;
    int m_y = 8;
    int d_x = 4;
    int d_y = 140;
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;

    cv::Rect roi(m_x,m_y,d_x,d_y);

    cv::resize( images->current_image,
                image,
                cv::Size(),
                scale_factor, scale_factor );

    cv::resize( images->previous_image(roi),
                templat,
                cv::Size(),
                scale_factor, scale_factor );

    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);

    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);

    int travel = (loc_max.x/scale_factor) - m_x;
    images->travel = travel*4;

    cv::Mat gray, sums, sobel;
    
    cv::cvtColor(images->current_image, gray, cv::COLOR_BGR2GRAY);

    cv::Sobel(gray, sobel, CV_64F, 0, 1, 7, 1, 0, cv::BORDER_DEFAULT);

    cv::reduce(sobel, sums, 1, cv::REDUCE_SUM );	

    cv::minMaxLoc(sums, &val_min, &val_max, &loc_min, &loc_max);

    int shift = -(((abs(loc_min.y - loc_max.y)/2)+loc_max.y)-115);

    float confidence = abs(val_max-val_min);

    float dV = 1e+06;
    
    if (confidence < dV)
        shift = images->shift_fallback;
    else
        images->shift_fallback = shift;

    images->shift = shift*4;

    return false;
}

bool loaded = false;

bool getMovement(Images *local_set){
    bool success = false;

    std::chrono::time_point<std::chrono::system_clock> start_comp;
    std::chrono::time_point<std::chrono::system_clock> end_comp;

    std::chrono::milliseconds dF = std::chrono::duration_cast<std::chrono::milliseconds>(local_set->c_stamp-local_set->p_stamp);
    local_set->frame_gap = dF.count();

    start_comp = std::chrono::system_clock::now();
    ///////////////////////////////////////////////////////////////////////////////////////
    if (loaded && dF.count() <=300){
        computeMovement(local_set);
        success = true;
    }
    local_set->previous_image = local_set->current_image.clone();
    local_set->p_stamp = local_set->c_stamp;
    loaded = true;
    ///////////////////////////////////////////////////////////////////////////////////////
    end_comp = std::chrono::system_clock::now();
    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);

    //std::cout << " dF: " << dF.count() << " dV: " << dV.count() << " " << std::endl;
    return success;
}

bool getMovement2(Images *local_set){
    bool success = false;

    std::chrono::time_point<std::chrono::system_clock> start_comp;
    std::chrono::time_point<std::chrono::system_clock> end_comp;

    std::chrono::milliseconds dF = std::chrono::duration_cast<std::chrono::milliseconds>(local_set->c_stamp-local_set->p_stamp);
    local_set->frame_gap = dF.count();

    start_comp = std::chrono::system_clock::now();
    ///////////////////////////////////////////////////////////////////////////////////////
    if (loaded && dF.count() <=300){
        success = computeMovement2(local_set);
    }
    local_set->previous_image = local_set->current_image.clone();
    local_set->p_stamp = local_set->c_stamp;
    loaded = true;
    ///////////////////////////////////////////////////////////////////////////////////////
    end_comp = std::chrono::system_clock::now();
    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);

    //std::cout << " dF: " << dF.count() << " dC: " << dV.count() << " " << std::endl;
    return success;
}