#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

int server_socket_id;
struct sockaddr_in server_socket;

void tcpStart(){
	server_socket_id = socket(AF_INET, SOCK_STREAM, 0);
	
	memset(&server_socket, 0, sizeof(server_socket));
	server_socket.sin_family = AF_INET;
    server_socket.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_socket.sin_port = htons(8079);
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 3000;
	setsockopt(server_socket_id, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv));
	
	std::cout << "connecting...";
	if (connect(server_socket_id, (struct sockaddr*)&server_socket, sizeof(server_socket)) == 0)
		std::cout << "connected" << std::endl;
	else
		std::cout << "failed" << std::endl;
}

void tcpUpdate(){
	std::chrono::time_point<std::chrono::system_clock> begin_time, end_time;
	begin_time = std::chrono::system_clock::now(); 

	std::stringstream tcpMessage;
	tcpMessage << tags.deviation << ':' << tags.speed << ':' << tags.program << ':' << tags.trim << "M";
	char formatted_message[tcpMessage.str().length()+1];
	strcpy(formatted_message, tcpMessage.str().c_str());

	write(server_socket_id, formatted_message, sizeof(formatted_message));
	
	char response[100];
	int number_of_bytes_read = read(server_socket_id, response, sizeof(response));

	if (number_of_bytes_read >= 3){
		tags.program = (int)response[1];
		tags.trim = (int)response[2]-127;
		//std::cout << tags.program << "\t" << tags.trim << std::endl;
	}
	if (response[0] != 'B'){
		close(server_socket_id);
		tcpStart();
	}

	end_time = std::chrono::system_clock::now();
    float loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-begin_time).count();
	//std::cout << "socket comms: " << loop_duration << std::endl;
}