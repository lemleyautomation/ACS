#include "Main.hpp"

Tags tags;
Images images;

#include "SocketClient.hpp"
#include "CameraInterface.hpp"
//#include "ModbusInterface.hpp"

void programLoop(){
    // a variable that allows us to measure the length of one program scan.
    std::chrono::time_point<std::chrono::system_clock> initial_time = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> begin_time, end_time;
    RollingAverage stop_watch;
    stop_watch.reset();
    stop_watch.base = 10;

    int image_error = 0;

    while (image_error >= 0){
        begin_time = std::chrono::system_clock::now(); 

        int image_error = get_new_image(camera_pointer, tags.module_number);

        tcpUpdate();
        images.program = tags.program;
        images.trim = int((((float)tags.trim/32.0)*(float)ppi)/4);
        
        if (!image_error){
            getMovement(&images);
            tags.deviation = (float)images.shift_average.update(images.shift)/(float)ppi;
            float inches_per_frame = (float)images.travel_average.update(images.travel)/(float)ppi;
            float inches_per_second = inches_per_frame/((float)images.frame_gap*0.001);
            float feet_per_minute = inches_per_second*5.0;
            float speed_percentage = abs( feet_per_minute / 200 );
            tags.speed = speed_percentage;
        }

        end_time = std::chrono::system_clock::now();
        float loop_duration = stop_watch.update(std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time).count());
        
        if (loop_duration > 150 && !stop_watch.startup()){
            std::cout << "loop too slow...";
            int program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-initial_time).count();
            if (program_duration > 10000){
                std::cout << "restarting." << std::endl;
                break;
            }
            std::cout << std::endl;
        }
    }
}

int main(int number_of_command_arguments, char **command_line_arguments){
    if (number_of_command_arguments > 2){
        tags.serial_number = command_line_arguments[2];
        tags.module_number = std::stoi(command_line_arguments[1]);
        std::cout << "Camera Serial Number: " << tags.serial_number << std::endl;
        std::cout << "Module Number: " << tags.module_number << std::endl;

        images.center_cam = camera_offsets[tags.module_number];
    }
    else{
        std::cout << "no module number or camera serial number specified." << std::endl;
        return -1;
    }

    if (!tcpStart()){
        return -1;
    }

    if (startCamera() != 0){
        std::cout << "error starting camera." << std::endl;
        stopCamera();
        return -1;
    }
   
    images.travel_average.base = 3;
    images.shift_average.base = 10;

    programLoop();

    stopCamera();
    tcpStop();
}
