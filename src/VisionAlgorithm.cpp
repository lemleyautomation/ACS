#include "Main.hpp"

void computeSpeed(Images *images){
    
    cv::Mat image, templat, heat_map;
    float scale_factor = 0.5;           // (320 X 156) -> (160 X 78)
    int m_x = 6;
    int m_y = 8;
    int d_x = 4;
    int d_y = 140;
    int margin_y = 40;

    cv::Rect i_roi(0, margin_y,images->current_image.cols,images->current_image.rows-(margin_y*2));
    cv::Rect p_roi(m_x,m_y,d_x,d_y);
    
    cv::resize( images->current_image(i_roi),
                image,
                cv::Size(),
                scale_factor, scale_factor );
    
    cv::resize( images->previous_image(p_roi),
                templat,
                cv::Size(),
                scale_factor, scale_factor );

    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;
    
    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    
    int travel = (loc_max.x/scale_factor) - m_x;
    images->travel = travel*4;
}

void computeMovement(Images *images){

    int offset = 0;
    int window_size = 40;
    int margin_x = 0;
    int margin_y = 40;
    
    computeSpeed(images);
    cv::Mat image, templat, heat_map;
    float scale_factor = 0.5;

    int corner = ((images->pattern_image.rows-window_size)/2)+offset;
    // roi = Region of Interest. Shaves off edges to improve time.
    cv::Rect i_roi(margin_x, margin_y,images->current_image.cols-(margin_x*2),images->current_image.rows-(margin_y*2));
    cv::Rect p_roi(0,corner,images->pattern_image.cols-1,window_size);
    
    cv::resize( images->current_image(i_roi),
                image,
                cv::Size(),
                scale_factor, 1 );
    
    cv::resize( images->pattern_image(p_roi),
                templat,
                cv::Size(),
                scale_factor, 1 );
    
    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;

    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    int shift = loc_max.y - corner;

    images->shift = shift * 4;
    
}

float oneDaverage(cv::Mat *vector){
    float sum = 0;
    float* beginning_of_row = vector->ptr<float>(0);
    for(int i = 0; i < vector->cols; i++)
        sum += beginning_of_row[i];
    return sum / vector->cols;
}

bool oneDthresh(cv::Mat *vector, float bound){
    
    float average = oneDaverage(vector);

    float upper_average = 0;
    float lower_average = 0;;

    float* beginning_of_row = vector->ptr<float>(0);
    for(int i = 0; i < vector->cols; i++){
        upper_average += beginning_of_row[i]*(beginning_of_row[i]>average);
        lower_average += beginning_of_row[i]*(beginning_of_row[i]<average);
    }
    upper_average /= vector->cols;
    lower_average /= vector->cols;

    float noise_band = abs(upper_average - lower_average);

    return (noise_band > bound);
}

void computeMovement2(Images *images){

    int offset = -13;

    computeSpeed(images);
    cv::Mat gray, sums, sobel;
    
    cv::cvtColor(images->current_image, gray, cv::COLOR_BGR2GRAY);

    cv::Sobel(gray, sobel, CV_64F, 0, 1, 7, 1, 0, cv::BORDER_DEFAULT);

    cv::reduce(sobel, sums, 1, cv::REDUCE_SUM );

    cv::normalize(sums, sums, 1, -1, cv::NORM_MINMAX);

    bool confidence = oneDthresh(&sums, 0.60);
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;

    cv::minMaxLoc(sums, &val_min, &val_max, &loc_min, &loc_max);

    int shift = -(((abs(loc_min.y - loc_max.y)/2)+loc_max.y)-(128+offset));

    if (confidence)
        images->shift_fallback = shift;
    else
        shift = images->shift_fallback;

    images->shift = shift*4;
}

bool loaded = false;

void getMovement(Images *local_set){
    std::chrono::time_point<std::chrono::system_clock> start_comp, end_comp;

    std::chrono::milliseconds dF = std::chrono::duration_cast<std::chrono::milliseconds>(local_set->c_stamp-local_set->p_stamp);
    local_set->frame_gap = dF.count();

    start_comp = std::chrono::system_clock::now();
    ///////////////////////////////////////////////////////////////////////////////////////
    if (loaded && dF.count() <=300){
        if (local_set->program == 1)
            computeMovement(local_set);
        else
            computeMovement2(local_set);
    }
    local_set->previous_image = local_set->current_image.clone();
    local_set->p_stamp = local_set->c_stamp;
    loaded = true;
    ///////////////////////////////////////////////////////////////////////////////////////
    end_comp = std::chrono::system_clock::now();

    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);
    std::cout << " dF: " << dF.count() << " dV: " << dV.count() << " " << std::endl;
}