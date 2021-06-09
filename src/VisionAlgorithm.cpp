#include "Main.hpp"

int coffset [] = {
    0,
    128,
    142,
    133,
    113,
    148,
    108, //114
    130,
    140,
    110
};

cv::Mat findAngles(cv::Mat image, float binning){
    cv::Mat sobelx, sobely, magnitudes, angles;

    cv::Sobel(image, sobelx, CV_32F, 1, 0, 1);
    cv::Sobel(image, sobely, CV_32F, 0, 1, 1);
    cv::cartToPolar(sobelx, sobely, magnitudes, angles, true);

    cv::resize(angles, angles, cv::Size(), binning, binning, cv::INTER_AREA );

    return angles;
}


double val_min, val_max;
cv::Point getPosition(cv::Mat &search){
    cv::Point loc_min, loc_max;
    cv::minMaxLoc(search, &val_min, &val_max, &loc_min, &loc_max);
    return loc_max;
}

bool confidence(Images *images, cv::Mat&result){
    float scale_factor = 0.5;
    cv::Mat heat_graph, image, templat;
    cv::resize( images->current_image, image, cv::Size(), scale_factor, scale_factor );
    cv::resize( images->previous_image, templat, cv::Size(), scale_factor, scale_factor );
    cv::matchTemplate(image, templat, heat_graph, cv::TM_CCOEFF_NORMED);
    float similarity = heat_graph.at<float>(0,0);

    int rows = result.rows;
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);
    cv::reduce(result, result, 0, cv::REDUCE_SUM);
    float noise = result.at<float>(0,0)/rows;
    noise = 1/(noise*10);

    float similarity_weight = 0.5;
    float noise_weight = 1;

    float confidence_value = (similarity*similarity_weight)+(noise*noise_weight);

    //std::cout << std::fixed;
    //std::cout << std::setprecision(2);
    //std::cout << "\t" << val_max << "\t" << similarity << "\t" << noise << "\t" << confidence_value << std::endl;

    // tufted: >0.4 >0.7

    if (images->program == 1)
        return (confidence_value>0.2);
    else
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

int stack_head = 0;
int stack_base = 7;
std::vector<cv::Mat> image_stack(stack_base);
cv::Mat stacked = cv::Mat::zeros(256, 320, CV_8UC3);
bool full_stack = false;

float pos = 50;

void computeMovement(Images *images){
    computeSpeed(images);

    cv::Mat angles, result, result1, result2, im, tm, sobely, sobelx, magnitudes;

    int center_cam = coffset[images->module_number];
    int window_offset = 0;
    cv::Point position;
    int shift = 0;

    //images->program = 4;

    if (images->program == 1){
        images->shift_average.base = 7;
        cv::Rect roi( 0, (center_cam)-25, images->pattern_image.cols, 50);
        cv::resize(images->current_image, im, cv::Size(), 0.5, 1, cv::INTER_LINEAR );
        cv::resize(images->pattern_image(roi), tm, cv::Size(), 0.5, 1, cv::INTER_LINEAR );
        cv::matchTemplate(im, tm, result, cv::TM_CCOEFF_NORMED);
        position = getPosition(result);
        window_offset = 25;
        shift = (position.y+window_offset);
        shift = -((shift-center_cam)*4);
        //std::cout << "Tufted" << std::endl;
    }
    else if (images->program == 2){
        images->shift_average.base = 1;

        cv::resize(images->current_image, im, cv::Size(100,100), 0,0, cv::INTER_AREA );
        //cv::GaussianBlur(im, im, cv::Size(0,0), 1);

        cv::Mat bgr[3];
        cv::split(im,bgr);

        cv::Mat Rx,Ry,Rm,Ra,Rs;
        cv::Mat Gx,Gy,Gm,Ga,Gs;
        cv::Mat Bx,By,Bm,Ba,Bs;

        cv::Sobel(bgr[0], Rx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[0], Ry, CV_32F, 0, 1, 1);
        cv::cartToPolar(Rx, Ry, Rm, Ra, true);

        cv::Sobel(bgr[1], Gx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[1], Gy, CV_32F, 0, 1, 1);
        cv::cartToPolar(Gx, Gy, Gm, Ga, true);

        cv::Sobel(bgr[2], Bx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[2], By, CV_32F, 0, 1, 1);
        cv::cartToPolar(Bx, By, Bm, Ba, true);

        cv::normalize(Rm,Rm, 0,255, cv::NORM_MINMAX);
        cv::normalize(Gm,Gm, 0,255, cv::NORM_MINMAX);
        cv::normalize(Bm,Bm, 0,255, cv::NORM_MINMAX);

        cv::normalize(Ra,Ra, 0,255, cv::NORM_MINMAX);
        cv::normalize(Ga,Ga, 0,255, cv::NORM_MINMAX);
        cv::normalize(Ba,Ba, 0,255, cv::NORM_MINMAX);

        Ra.convertTo(Ra, CV_8U);
        Ga.convertTo(Ga, CV_8U);
        Ba.convertTo(Ba, CV_8U);
        Rm.convertTo(Rm, CV_8U);
        Gm.convertTo(Gm, CV_8U);
        Bm.convertTo(Bm, CV_8U);

        uint8_t lt[256];
        for (int i =0; i < 256; i++){
            float datum = (-128*cos(i*0.049))+128;
            lt[i] = uint8_t(datum);
        }
        cv::Mat lut(256, 1, CV_8U, 1);

        cv::LUT(Ra,lut,Ra);
        cv::LUT(Ga,lut,Ga);
        cv::LUT(Ba,lut,Ba);
        
        result = (0.2*Rm) + (0.2*Gm) + (0.2*Bm); // simple tufted

        //cv::GaussianBlur(result, result, cv::Size(0,0), 1);

        cv::reduce(result,result,1,cv::REDUCE_AVG);

        position = getPosition(result);

        if (position.y > pos)
            pos++;
        else if (position.y < pos)
            pos--;
        
        if (images->travel_average.avg < 50)
            pos = int((float)center_cam/2.56);

        window_offset = 0;

        shift = int(pos*2.56) + window_offset;
        //std::cout << "\t" << position.y << "\t" << shift << "\n";
        shift = -((shift-(center_cam))*4);

        images->shift = shift;
    }
    else{
        images->shift_average.base = 1;

        cv::resize(images->current_image, im, cv::Size(100,100), 0,0, cv::INTER_AREA );
        cv::GaussianBlur(im, im, cv::Size(0,0), 1);

        cv::Mat bgr[3];
        cv::split(im,bgr);

        cv::Mat Rx,Ry,Rm,Ra,Rs;
        cv::Mat Gx,Gy,Gm,Ga,Gs;
        cv::Mat Bx,By,Bm,Ba,Bs;

        cv::Sobel(bgr[0], Rx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[0], Ry, CV_32F, 0, 1, 1);
        cv::cartToPolar(Rx, Ry, Rm, Ra, true);

        cv::Sobel(bgr[1], Gx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[1], Gy, CV_32F, 0, 1, 1);
        cv::cartToPolar(Gx, Gy, Gm, Ga, true);

        cv::Sobel(bgr[2], Bx, CV_32F, 1, 0, 1);
        cv::Sobel(bgr[2], By, CV_32F, 0, 1, 1);
        cv::cartToPolar(Bx, By, Bm, Ba, true);

        cv::normalize(Rm,Rm, 0,255, cv::NORM_MINMAX);
        cv::normalize(Gm,Gm, 0,255, cv::NORM_MINMAX);
        cv::normalize(Bm,Bm, 0,255, cv::NORM_MINMAX);

        cv::normalize(Ra,Ra, 0,255, cv::NORM_MINMAX);
        cv::normalize(Ga,Ga, 0,255, cv::NORM_MINMAX);
        cv::normalize(Ba,Ba, 0,255, cv::NORM_MINMAX);

        Ra.convertTo(Ra, CV_8U);
        Ga.convertTo(Ga, CV_8U);
        Ba.convertTo(Ba, CV_8U);
        Rm.convertTo(Rm, CV_8U);
        Gm.convertTo(Gm, CV_8U);
        Bm.convertTo(Bm, CV_8U);

        uint8_t lt[256];
        for (int i =0; i < 256; i++){
            float datum = (-128*cos(i*0.049))+128;
            lt[i] = uint8_t(datum);
        }
        cv::Mat lut(256, 1, CV_8U, 1);

        cv::LUT(Ra,lut,Ra);
        cv::LUT(Ga,lut,Ga);
        cv::LUT(Ba,lut,Ba);
        
        result = (0.32*Ra) + (0.167*Ga) + (0.167*Ba) + (0.167*Rm) + (0.167*Gm) + (0.167*Bm);

        //cv::GaussianBlur(result, result, cv::Size(0,0), 1);

        cv::reduce(result,result,1,cv::REDUCE_AVG);

        position = getPosition(result);

        if (position.y > pos)
            pos++;
        else if (position.y < pos)
            pos--;
        
        if (images->travel_average.avg < 50)
            pos = int((float)center_cam/2.56);

        window_offset = 10;

        shift = int(pos*2.56) + window_offset;
        std::cout << "\t" << position.y << "\t" << shift << "\n";
        shift = -((shift-(center_cam))*4);

        images->shift = shift;
    }

    //std::cout << shift << std::endl;

    if (true)//confidence(images,result))
        images->shift = shift;
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