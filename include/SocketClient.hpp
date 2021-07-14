#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

int server_socket_id;
struct sockaddr_in server_socket;

bool tcpStart(){
	server_socket_id = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket_id == -1){
		std::cout << "socket creation failed" << std::endl;
		return false;
	}
	
	memset(&server_socket, 0, sizeof(server_socket));
	server_socket.sin_family = AF_INET;
    server_socket.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_socket.sin_port = htons(8079);

	int connection_status = connect(server_socket_id, (struct sockaddr*)&server_socket, sizeof(server_socket));
	if (connection_status == -1){
		std::cout << "socket connection failed" << std::endl;
		return false;
	}

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 3000;
	if (setsockopt(server_socket_id, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
		perror("Error");
	}

	return true;
}

void tcpStop(){
	close(server_socket_id);
}
void tcpReset(){
	tcpStop();
	tcpStart();
}

void tcpUpdate(){

	std::stringstream tcpMessage;
	tcpMessage << tags.module_number << ':' << tags.deviation << ':' << tags.speed << ':' << tags.program << ':' << tags.trim << "M";
	char formatted_message[tcpMessage.str().length()+1];
	strcpy(formatted_message, tcpMessage.str().c_str());

	//std::cout << tcpMessage.str() << std::endl;
	
	write(server_socket_id, formatted_message, sizeof(formatted_message));
	
	char response[100];
	int number_of_bytes_read = read(server_socket_id, response, sizeof(response));

	if (number_of_bytes_read >= 3){
		tags.program = int(response[1]);
		if (int(response[2]) > 128){
			tags.trim = int(response[2]) - 256;
		}
		else{
			tags.trim = int(response[2]);
		}
		//std::cout << tags.program << "\t" << tags.trim << std::endl;
	}
	else if (number_of_bytes_read <= 0){
		std::cout << "lost connection, resetting." << std::endl;
		tcpReset();
	}
}