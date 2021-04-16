#include "Main.hpp"

Tags tags;
Images images;
moduleSettings mset;

#include "SocketClient.hpp"
#include "CameraInterface.hpp"
#include "ModbusInterface.hpp"
#include "SocketServer.hpp"

void printdiagnostics(){
    std::bitset<16> servo_status_word_1(_3x[44]);
    std::bitset<16> servo_status_word_2(_3x[45]);
    std::bitset<16> servo_status_word_3(_3x[46]);
    std::bitset<16> servo_status_word_4(_3x[47]);
    //std::bitset<16> servo_command_word_1(_4x[0]);
    //std::bitset<16> servo_command_word_3(_4x[2]);
    std::cout << servo_status_word_1 << "\t";
    std::cout << servo_status_word_2 << "\t";
    std::cout << servo_status_word_3 << "\t";
    std::cout << servo_status_word_4 << "\t";
    //std::cout << servo_command_word_1 << "\t";
    //std::cout << servo_command_word_3 << "\t";
    //std::cout << toFloat(_4x[37],_4x[36]) << " " << toFloat(_3x[49], _3x[48]) << " " << tags.deviation << " " << images.shift;
    std::cout << (int)tags.speed << "\t" << toFloat(_4x[21], _4x[20]) << "\t" << toFloat(_4x[29], _4x[28]);
    std::cout << std::endl;
}

void programLoop(){
    // a variable that allows us to measure the length of one program scan.
    RollingAverage stop_watch;
    stop_watch.reset();
    stop_watch.base = 10;

    int image_error = 0;
    // all the code inside the while will loop over and over until the camera has a fatal error
    while (image_error >= 0){
        // start keeping track of how long it takes to do one program scan
        begin_time = std::chrono::system_clock::now(); 
        // take a picture
        int image_error = get_new_image(camera_pointer, mset.module_number);
        // passes camera error status on to the tag server
        tags.cam_status = (image_error==0);
        // read all the status registers in the servo
        modbus_read_input_registers(ctx_servo, 0, 82, _3x);
        // passes the servo ready bit on to the tag server
        tags.drive_status = getBit(_3x, 45, 3);
        // read the status of the limit switch
        checkSwitch(mset);
        // if the camera has a non-fatal error, the 'continue' statement skips all the code and starts the loop again.
        if (!image_error){
            // process the picture through the vision algorithm
            getMovement2(&images);
            // update the speed and deviation measurements based on the processed image
            tags.deviation = ((float)images.shift_average.update(images.shift)/(float)ppi);
            tags.speed = varySpeed(images.travel_average.update(images.travel), images.frame_gap);
            tags.underspeed = (tags.speed < 0.25);
        }
        // update the servo registers based on the speed, devation, and limit switch
        updateServo();
        // print to the console the frame-to-frame measurments register statuses
        //printdiagnostics(dev);    // LEAVE THIS COMMENTED OUT UNLESS YOU NEED IT. IF THIS IS UN-COMMENTED, IT WILL USE ALL THE MEMORY ON THE PI IN ABOUT 2 DAYS.
        // calulate the time for one program scan in milliseonds
        end_time = std::chrono::system_clock::now();
        float loop_duration = stop_watch.update(std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time).count());
        //std::cout << loop_duration << std::endl; //  LEAVE THIS COMMENTED OUT UNLESS YOU NEED IT. SEE ABOVE ^
        // if the loop is going to slow, this could be caused by communications errors. restarting the programs fixes almost all of these
        if (loop_duration > 150 && !stop_watch.startup()){
            std::cout << "loop too slow, restarting" << std::endl;
            break;
        }
    }
}

int main(int argc, char **argv){
    if (argc > 2){
        mset.serial_number = argv[2];
        mset.module_number = std::stoi(argv[1]);
        tags.module_number = std::stoi(argv[1]);
        std::cout << "Camera Serial Number: " << mset.serial_number << std::endl;
        std::cout << "Module Number: " << mset.module_number << std::endl;
    }
    else{
        std::cout << "no module number or camera serial number specified." << std::endl;
        return -1;
    }

    startMessaging();

    if (startModbusInterfaces(mset) != 0){
        std::cout << "error starting Modbus Driver." << std::endl;
        stopModbus(mset);
        return -1;
    }
    tags.status = true;

    if (startCamera(mset) != 0){
        std::cout << "error starting camera." << std::endl;
        stopCamera();
        stopModbus(mset);
        return -1;
    }
    tags.cam_status = true;

    if (configure_servo() != 0){
        std::cout << "error configuring servo." << std::endl;
        stopCamera();
        stopModbus(mset);
        return -1;
    }
    else{
        std::cout << "servo configured successfully" << std::endl;
    }
    tags.drive_status = true;

    images.travel_average.base = 3;
    images.shift_average.base = 8;

    programLoop();

    stopCamera();
    stopModbus(mset);
    stopMessaging();
}
