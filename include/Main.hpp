#ifndef _INCL_GUARD
#define _INCL_GUARD

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <bitset>
#include <chrono>
#include <cmath>
#define PI 3.14159265

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

#include "dirent.h"

#include "spinnaker/Spinnaker.h"
#include "spinnaker/SpinGenApi/SpinnakerGenApi.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

#include <modbus.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

const int ppi = 300;

static int camera_offsets [] = {
    0,
    128,
    142,
    140,
    113,
    128,
    128,
    130,
    140,
    110
};

struct RollingAverage{
    int samples[100];
    int head = 0;
    int base = 1;
    float avg = 0;
    int start_count = 0;

    void reset(){
        for (int i = 0; i < base; i++){
            samples[i] = 0;
        }
    }
    float update(int sample){
        if (start_count <= base)
            start_count++;

        samples[head] = sample;

        head = (head+1)%base;

        float average = 0;

        for(int i = 0; i < base; i++){
            average += (float)samples[i];
        }
        average = average / (float)base;
        avg = average;
        return average;
    }

    bool startup(){
        return (start_count <= base);
    }
};

struct Images{
    cv::Mat current_image;
    cv::Mat previous_image;
    cv::Mat pattern_image;
    std::chrono::time_point<std::chrono::system_clock> c_stamp;
    std::chrono::time_point<std::chrono::system_clock> p_stamp;

    int center_cam;
    int program = 2;
    int trim = 0;

    int shift;
    RollingAverage shift_average;

    int travel;
    RollingAverage travel_average;

    int frame_gap;
};

struct Tags{
    float deviation = 0;
    float speed = 0;
    bool shutdown = false;
    
    int program = 2;
    int trim = 0;

    int module_number = 0;
    const char *servos[9] = { "192.168.1.31",
                                "192.168.1.32",
                                "192.168.1.33",
                                "192.168.1.34",
                                "192.168.1.35",
                                "192.168.1.36",
                                "192.168.1.37",
                                "192.168.1.38",
                                "192.168.1.39"};
    std::string serial_number;

    std::string serialize(){
        std::stringstream tcpMessage;
        tcpMessage << "{\"deviation\":" << deviation << ',';
        tcpMessage << "\"speed\":" << speed << ',';
        tcpMessage << "\"program\":" << program << ',';
        tcpMessage << "\"trim\":" << trim << '}';
        return tcpMessage.str();
    }
    void deserialize(std::string tcpMessage){
        rapidjson::Document document;
        document.Parse((char*)tcpMessage.c_str());

        if (document.IsObject()){
            rapidjson::Value& program_command_tag = document["program command"];
            program = program_command_tag.GetInt();

            if (program > 3 || program < 1){
                program = 2;
            }

            rapidjson::Value& trim_command_tag = document["trim command"];
            trim = trim_command_tag.GetInt();

            if (trim < -32 || trim > 32){
                trim = 0;
            }
        }
    }
};

void getMovement(Images *local_set);
#endif

//wut
