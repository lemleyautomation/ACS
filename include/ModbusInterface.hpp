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

float varySpeed(float travel, int frame_gap){
    float speed_factor = ((((float)travel/(float)ppi)/((float)frame_gap*0.001))*5)/200;
    //std::cout.precision(2);
    //std::cout << std::fixed;
    //std::cout << abs(speed_factor) << std::endl;
    float acceleration = 40.0*speed_factor;
    float deceleration = 40.0*speed_factor;
    float servo_speed  =  2.0*speed_factor;
    toUint(_4x, 21, acceleration); // accel
    toUint(_4x, 25, deceleration); // decel
    toUint(_4x, 29,  servo_speed); // speed

    return speed_factor;
}

bool switchRisingEdge(){
    if (status != prev_status && status == true){
        setBit(_4x, 1, 0, 1); // enable servo
        return true;
    }
    else if (status == false){
        setBit(_4x, 1, 0, 0); // disable servo
    }

    return false;
}

void checkSwitch(moduleSettings mset){
    modbus_read_registers(ctx_switch, 31, 1, &limit_switches);
    std::bitset<16> stride_word(limit_switches);
    prev_status = status;
    status = stride_word[mset.module_number-1];
}