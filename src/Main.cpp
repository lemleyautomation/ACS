#include "Main.hpp"
#include "ModbusInterface.hpp"
#include "CameraInterface.hpp"
#include "ModbusServer.hpp"

//#define ASIO_STANDALONE
//#include <asio.hpp>
//#include <asio/ts/buffer.hpp>
//#include <asio/ts/internet.hpp>

void printdiagnostics(float dev){
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
    //std::cout << toFloat(_4x[37],_4x[36]) << " " << toFloat(_3x[49], _3x[48]) << " " << dev << " " << images.shift;
    std::cout << (int)speed.avg << "\t" << toFloat(_4x[21], _4x[20]) << "\t" << toFloat(_4x[29], _4x[28]);
    std::cout << std::endl;
}

void programLoop(){
    RollingAverage loop_duration;
    loop_duration.reset();
    loop_duration.base = 10;

    Tags local_set;
    std::thread tag_server(tagServer);

    //bool toggle = false;
    int toggle = 0;
    while (true){
        begin_time = std::chrono::system_clock::now(); 
        local_set.cam_status = 1;
        int image_error = get_new_image(camera_pointer, mset.module_number);
        if (image_error == -1005 || image_error == -1002 || image_error == -1012 || image_error == -1010){
            local_set.cam_status = 0;
            std::cout << "restarting camera" << std::endl;
            stopCamera();
            startCamera(mset);
        }
        else if (image_error){
            local_set.cam_status = 0;
            std::cout << image_error << std::endl;
            continue;
        }
        
        modbus_read_input_registers(ctx_servo, 0, 82, _3x);

        if (switchRisingEdge()){
            newPattern();
            deviation.reset();
        }
        
        getMovement(&images);
        float dev = ((float)deviation.update(images.shift)/(float)ppi);
        local_set.deviation = dev;

        //printdiagnostics(dev);

        float spd_fc = varySpeed(speed.update(images.travel), images.frame_gap);
        local_set.speed = spd_fc;
        local_set.underspeed = (spd_fc < 0.25);

        if (!toggle){
            setBit(_4x, 1, 3, 0); // set start move to low
        }
        else if (toggle ==  3){
            if (abs(dev)> 0.03 && status && getBit(_3x, 45, 3)){
                setBit(_4x, 1, 3, 1); // set start move to high
            }
            //***********************************************************************************//
            float servo_position_command = toFloat(_3x[49], _3x[48]) + (dev);                    // actual control
            toUint(_4x, 37, servo_position_command);                                             //
            //***********************************************************************************//
            checkSwitch(mset);
        }
        toggle = (toggle+1)%4;

        setBit(_4x, 3, 0, getBit(_3x, 45, 0)); // heartbeat
        modbus_write_registers(ctx_servo, 0, 54, _4x);

        local_set.status = 1;
        local_set.cam_status = 1;
        local_set.drive_status = 1;
        setTags(local_set);

        end_time = std::chrono::system_clock::now();
        float c_duration = loop_duration.update(std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time).count());
        //std::cout << c_duration << std::endl;

        if (c_duration > 150 && !loop_duration.pause()){
            std::cout << "loop too slow, restarting" << std::endl;
            break;
        }
    }

    local_set.shutdown = true;
    setTags(local_set);
    tag_server.join();
}

int main(int argc, char **argv){
    if (argc > 2){
        mset.serial_number = argv[2];
        mset.module_number = std::stoi(argv[1]);
        std::cout << "Camera Serial Number: " << mset.serial_number << std::endl;
        std::cout << "Module Number: " << mset.module_number << std::endl;
    }
    else{
        std::cout << "no module number or camera serial number specified." << std::endl;
        return -1;
    }

    if (startModbusInterfaces(mset) != 0){
        std::cout << "error starting Modbus Driver." << std::endl;
        stopModbus(mset);
        return -1;
    }

    if (startCamera(mset) != 0){
        std::cout << "error starting camera." << std::endl;
        stopCamera();
        stopModbus(mset);
        return -1;
    }

    if (configure_servo() != 0){
        std::cout << "error configuring servo." << std::endl;
        stopCamera();
        stopModbus(mset);
        return -1;
    }
    else{
        std::cout << "servo configured successfully" << std::endl;
    }

    speed.base = 3;
    deviation.base = 8;

    programLoop();

    stopCamera();
    stopModbus(mset);
}
