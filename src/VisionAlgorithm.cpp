#include "Main.hpp"

double val_min, val_max;
cv::Point getPosition(cv::Mat &search){
    cv::Point loc_min, loc_max;
    cv::minMaxLoc(search, &val_min, &val_max, &loc_min, &loc_max);
    return loc_max;
}

void processChanel(cv::Mat channel, cv::Mat& magnitude, cv::Mat& angle){
    cv::Mat x,y;
    cv::Sobel(channel, x, CV_32F, 1, 0, 1);
    cv::Sobel(channel, y, CV_32F, 0, 1, 1);
    cv::cartToPolar(x, y, magnitude, angle, true);
    cv::normalize(magnitude, magnitude, 0,255, cv::NORM_MINMAX);
    cv::normalize(angle, angle, 0,255, cv::NORM_MINMAX);
    magnitude.convertTo(magnitude, CV_8U);
    angle.convertTo(angle, CV_8U);
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

float pos = 50;

void computeMovement(Images *images){
    computeSpeed(images);

    cv::Mat result, im, tm;

    cv::Mat bgr[3];
    cv::Mat Rm,Ra;
    cv::Mat Gm,Ga;
    cv::Mat Bm,Ba;

    cv::Point position;
    int shift = 0;

    if (images->program == 1){
        images->shift_average.base = 7;
        cv::Rect roi( 0, (images->center_cam)-25, images->pattern_image.cols, 50);
        cv::resize(images->current_image, im, cv::Size(), 0.5, 1, cv::INTER_LINEAR );
        cv::resize(images->pattern_image(roi), tm, cv::Size(), 0.5, 1, cv::INTER_LINEAR );
        cv::matchTemplate(im, tm, result, cv::TM_CCOEFF_NORMED);
        position = getPosition(result);
        shift = (position.y+images->trim);
        shift = -((shift-images->center_cam)*4);
        images->shift = shift;
        //std::cout << "Tufted" << std::endl;
    }
    else if (images->program == 2){
        images->shift_average.base = 4;

        cv::resize(images->current_image, im, cv::Size(100,100), 0,0, cv::INTER_AREA );
        
        cv::split(im,bgr);

        processChanel(bgr[0],Rm,Ra);
        processChanel(bgr[1],Gm,Ga);
        processChanel(bgr[2],Bm,Ba);

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
            pos = int((float)images->center_cam/2.56);

        shift = int(pos*2.56) + images->trim;
        //std::cout << "\t" << position.y << "\t" << shift << "\n";
        shift = -((shift-(images->center_cam))*4);

        images->shift = shift;
    }
    else{
        images->shift_average.base = 4;

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
        Rm.convertTo(Rm, CV_64F);
        Gm.convertTo(Gm, CV_64F);
        Bm.convertTo(Bm, CV_64F);

        uint8_t lt[256];
        for (int i =0; i < 256; i++){
            float datum = (-128*cos(i*0.049))+128;
            lt[i] = uint8_t(datum);
        }
        cv::Mat lut(256, 1, CV_8U, 1);

        cv::LUT(Ra,lut,Ra);
        cv::LUT(Ga,lut,Ga);
        cv::LUT(Ba,lut,Ba);
        
        Rm.convertTo(Ra, CV_64F);
        Gm.convertTo(Ga, CV_64F);
        Bm.convertTo(Ba, CV_64F);
        
        result = (Rm*Ra) + (Gm*Ga) + (Bm*Ba);

        //cv::GaussianBlur(result, result, cv::Size(0,0), 1);

        cv::reduce(result,result,1,cv::REDUCE_AVG);

        int margin = 5;
        cv::Rect edge_trim( 0, margin, result.cols, result.rows-margin);
        cv::Mat r1 = result(edge_trim).clone();

        position = getPosition(r1);

        position.y += (margin*2);

        if (position.y > pos)
            pos++;
        else if (position.y < pos)
            pos--;
        
        if (images->travel_average.avg < 50)
            pos = int((float)images->center_cam/2.56);

        //shift = int(position.y*2.56) + window_offset;
        shift = int(pos*2.56) + images->trim;
        //std::cout << "\t" << position.y << "\t" << shift << "\t" << pos << "\n";
        shift = -((shift-(images->center_cam))*4);

        images->shift = shift;
    }

    //images->shift += images->trim;
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

    //std::cout << local_set->shift << "\t" << local_set->travel << "\t";

    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);
    //std::cout << " dF: " << dF.count() << " dC: " << dV.count() << " " << local_set->program << std::endl;
}