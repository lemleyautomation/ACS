std::chrono::time_point<std::chrono::system_clock> time_since_move, begin_time, end_time;

modbus_t *ctx_switch;
modbus_t *ctx_servo;

uint16_t limit_switches;
bool status;
bool prev_status;

uint16_t _3x[128];
uint16_t _4x[128];

static float toFloat(uint16_t upper_word, uint16_t lower_word){
    int32_t pl = ((uint32_t)upper_word << 16) + (uint16_t)lower_word;
    return (float)pl*0.01;
}
static void toUint(uint16_t* array, int reg, float number){
    number *= 100;

    uint16_t upper_word = ((int16_t)number >> 16);
    uint16_t lower_word = ((int16_t)number);

    array[reg] = upper_word;
    array[reg-1] = lower_word;
}

static void setBit(uint16_t* array, int reg, int bit, int value){
    std::bitset<16> word(array[reg-1]);
    word[bit] = value;
    array[reg-1] = word.to_ulong();
}
static bool getBit(uint16_t* array, int reg, int bit){
    std::bitset<16> word(array[reg-1]);
    return word[bit];
}

int startModbusInterfaces(moduleSettings mset){
    ctx_switch = modbus_new_tcp("192.168.1.100", 502);
    if ( modbus_connect(ctx_switch) != 0){
        std::cout << "limit switch connection failure" << std::endl;
        return -1;
    }

    ctx_servo = modbus_new_tcp(mset.servos[mset.module_number-1], 502);
    if ( modbus_connect(ctx_servo) != 0){
        std::cout << "servo connection failure" << std::endl;
        return -1;
    }

    return 0;
}
void stopModbus(moduleSettings mset){
    modbus_close(ctx_servo);
    modbus_free(ctx_servo);
    
    modbus_close(ctx_switch);
    modbus_free(ctx_switch);

    std::cout << "Modbus driver stopped." << std::endl;
}

int configure_servo(){
    std::cout << "configuring servo...";
    int x[4];
    x[0] = modbus_read_input_registers(ctx_servo, 0, 54, _3x) - 54;
    std::cout << x[0] << "...";
    x[1] = modbus_read_registers(ctx_servo, 0, 54, _4x) - 54;
    std::cout << x[1] << "...";

    toUint(_4x, 21, 4.0); // accel
    toUint(_4x, 25, 4.0); // decel
    toUint(_4x, 29,  2.0); // speed
    _4x[48] = 2;
    _4x[51] = 1;

    setBit(_4x,1,2,1);
    setBit(_4x,1,0,0);

    _4x[49] = 1; // homing type
    _4x[18] = _3x[48]; // ...
    _4x[19] = _3x[49]; // home position to current position

    x[2] = modbus_write_registers(ctx_servo, 0, 54, _4x) - 54;
    std::cout << x[2] << "...";

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    setBit(_4x, 1, 2, 0);
    setBit(_4x, 1, 0, 0);
    x[3] = modbus_write_registers(ctx_servo, 0, 54, _4x) - 54;
    std::cout << x[3] << "...";
    time_since_move = std::chrono::system_clock::now();

    return x[0] + x[1] + x[2] + x[3];
}

int toggle = 0;
void updateServo(){
    if (!toggle){
        setBit(_4x, 1, 3, 0); // set start move to low
    }
    else if (toggle ==  2){
        if (abs(tags.deviation) > 0.03 && tags.cam_status && !tags.underspeed && status == 1 && getBit(_3x, 45, 3)){
            setBit(_4x, 1, 3, 1); // set start move to high
        }
        //***********************************************************************************//
        float servo_position_command = toFloat(_3x[49], _3x[48]) - (tags.deviation);         // actual control
        toUint(_4x, 37, servo_position_command);                                             //
        //***********************************************************************************//
    
        sendMessage(tags);
    }
    toggle = (toggle+1)%3;

    setBit(_4x, 3, 0, getBit(_3x, 45, 0)); // heartbeat

    modbus_write_registers(ctx_servo, 0, 54, _4x);
}

float varySpeed(float travel, int frame_gap){
    // travel is measured in pixels of movement from one picture to the next picture
    // ppi = pixels per inch
    // frame_gap = the number of milliseconds between pictures
    // 1 millisecond = *0.001 seconds
    // 1 foot per minute = 5 inches per second
    // 200 feet per minute is the assumed maximum speed of the rollup
    // this way, speed_factor is always between 0 and 1. a percentage %
    float speed_factor = ((((float)travel/(float)ppi)/((float)frame_gap*0.001))*5)/200;
    //speed_factor = 1; // FOR TESTING PURPOSES
    // based on trial and erro, the acce/decel at 200 feet per minute should be 4 inches / second / second
    float acceleration = 4;//4.0*speed_factor;
    float deceleration = 4;//4.0*speed_factor;
    float servo_speed  =  2;//2.0*speed_factor;
    toUint(_4x, 21, acceleration); // accel
    toUint(_4x, 25, deceleration); // decel
    toUint(_4x, 29,  servo_speed); // speed

    return speed_factor;
}

void checkSwitch(moduleSettings mset){
    // read the limit switch register
    modbus_read_registers(ctx_switch, 31, 1, &limit_switches);
    // break the register word into individual bits
    std::bitset<16> stride_word(limit_switches);
    // this allow us to check for the moment the switch is made
    prev_status = status;
    // set the module status based on the proper bit
    status = stride_word[mset.module_number-1];
    setBit(_4x, 1, 0, status);
    // if the switch has just been made, then this statement is true
    if (status != prev_status && status == true){
        setBit(_4x, 1, 0, 1);           // enable servo
        newPattern();                   // acquire and new pattern image
        images.shift_average.reset();   // reset the rolling average for deviation to 0
    }
}
