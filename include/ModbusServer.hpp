#define rr registers->tab_registers

struct Tags{
    float deviation = 0.0;
    float speed = 0.0;
    bool underspeed = true;
    bool status = true;
    bool cam_status = false;
    bool drive_status = false;
    bool shutdown = false;
};

std::mutex lock_tags;
Tags tags;
modbus_mapping_t *registers;

void setTags(Tags local_set){
    {
        std::unique_lock<std::mutex> lck(lock_tags);
        tags.status = local_set.status;
        tags.cam_status  = local_set.cam_status;
        tags.drive_status = local_set.drive_status;
        tags.underspeed = local_set.underspeed;
        tags.deviation = local_set.deviation;
        tags.speed = local_set.speed;
        tags.shutdown = local_set.shutdown;
    }
}

bool updateRegisters(){
    bool shutdown = false;
    {
        std::unique_lock<std::mutex> lck(lock_tags);
        std::bitset<16> status_bits(rr[0]);
        status_bits.set(0, tags.status);
        status_bits.set(1, tags.cam_status);
        status_bits.set(2, tags.drive_status);
        status_bits.set(3, tags.underspeed);
        status_bits.set(15, tags.shutdown);
        shutdown = tags.shutdown;
        rr[0] = status_bits.to_ulong();
        rr[1] = (65536*(tags.deviation<0))+(tags.deviation*1000);
        rr[2] = (65536*(tags.speed<0))+(tags.speed*1000);
    }
    return shutdown;
}

void tagServer(){
    // create new Modbus communications context
    modbus_t *context = modbus_new_tcp(NULL, 512);
    // create registers. Only the 4x registers are used
    registers = modbus_mapping_new(0, 0, 5, 0);
    // begin listening for any income modbus communication
    int socket_id = modbus_tcp_listen(context, 1);
    while (true){
        // accept any communications request
        modbus_tcp_accept(context, &socket_id);
        // variable named 'query' that stores the incomiing request
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        // function call to recieve incoming request
        int query_size = modbus_receive(context, query);
        // the only way we it was a real request is to check the size
        if (query_size > 0) {
            // respond to request with register values
            modbus_reply(context, query, query_size, registers);
            //std::cout << "ping" << std::endl;
            // update the tag values and possibly shutdown comms.
            if (updateRegisters())
                break;
        }
    }
    // print reason for coms shutdown to console
    printf("tag server shutdown: %s\n", modbus_strerror(errno));
    // these function calls prevent memory leaks
    modbus_mapping_free(registers);
    modbus_close(context);
    modbus_free(context);
}