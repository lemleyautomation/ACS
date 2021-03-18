#include "Main.hpp"
#include "ModbusInterface.hpp"
#include "CameraInterface.hpp"

void printdiagnostics(float dev){
    std::bitset<16> servo_status_word_1(_3x[44]);
    std::bitset<16> servo_status_word_2(_3x[45]);
    std::bitset<16> servo_status_word_3(_3x[46]);
    std::bitset<16> servo_status_word_4(_3x[47]);
    std::bitset<16> servo_command_word_1(_4x[0]);
    std::bitset<16> servo_command_word_3(_4x[2]);
    std::cout << servo_status_word_1 << "\t";
    std::cout << servo_status_word_2 << "\t";
    std::cout << servo_status_word_3 << "\t";
    std::cout << servo_status_word_4 << "\t";
    std::cout << servo_command_word_1 << "\t";
    std::cout << servo_command_word_3 << "\t";
    std::cout << toFloat(_4x[37],_4x[36]) << " " << toFloat(_3x[49], _3x[48]) << " " << dev << " " << images.shift;
    //std::cout << dev;//speed.update(images.travel);
    std::cout << std::endl;
}

void programLoop(){
    RollingAverage loop_duration;
    loop_duration.reset();
    loop_duration.base = 10;

    while (true){
        begin_time = std::chrono::system_clock::now(); 
        if (get_new_image(camera_pointer))
            continue;
        

        modbus_read_input_registers(ctx_servo, 0, 82, _3x);

        if (checkSwitch()){
            newPattern();
            deviation.reset();
        }
        
        getMovement(&images);
        float dev = (float)deviation.update(images.shift)/(float)ppi;

        //printdiagnostics(dev);

        std::chrono::milliseconds dwell_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-time_since_move);
        if (dwell_time.count() > 30 && dwell_time.count() < 90){
            setBit(_4x, 1, 3, 0); // set start move to low
            modbus_write_registers(ctx_servo, 0, 54, _4x);
        }
        else if (dwell_time.count() >= 100){
            time_since_move = std::chrono::system_clock::now(); 

            if (abs(dev)> 0.03 && status && getBit(_3x, 45, 3)){
                setBit(_4x, 1, 3, 1); // set start move to high
            }
            float servo_position_command = toFloat(_3x[49], _3x[48]) + dev;
            toUint(_4x, 37, servo_position_command);

            modbus_read_registers(ctx_switch, 31, 1, &limit_switches);
            std::bitset<16> stride_word(limit_switches);
            prev_status = status;
            status = stride_word[mset.module_number-1];

            /*
            std::bitset<16> hmi_word(hmi[0]);
            hmi_word[0] = 1;
            hmi_word[1] = status;
            hmi_word[14] = hmi_word[15];
            hmi_word[15] = !hmi_word[15];
            hmi[0] = hmi_word.to_ulong();
            hmi[1] = abs((dev+10)*100);
            modbus_write_registers(ctx_hmi, 2*(mset.module_number-1), 2, hmi );
            */
            
            setBit(_4x, 3, 0, getBit(_3x, 45, 0)); // heartbeat
            modbus_write_registers(ctx_servo, 0, 54, _4x);
        }

        end_time = std::chrono::system_clock::now();
        float c_duration = loop_duration.update(std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time).count());
        std::cout << c_duration << std::endl;

        if (c_duration > 150){
            std::cout << "loop too slow, restarting" << std::endl;
            break;
        }
    }
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

    speed.base = 35;
    deviation.base = 14;

    programLoop();

    stopCamera();
    stopModbus(mset);
}
