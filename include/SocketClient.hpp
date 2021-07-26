#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

struct socketData{
	int socket;
	struct sockaddr_in server_address;
};

socketData configure_socket(std::string IP_address, int PORT){
	socketData socket_data;

	socket_data.socket = socket(AF_INET, SOCK_STREAM, 0);
	
	memset(&socket_data.server_address, 0, sizeof(socket_data.server_address));
	socket_data.server_address.sin_family = AF_INET;
    socket_data.server_address.sin_addr.s_addr = inet_addr((char*)IP_address.c_str());
    socket_data.server_address.sin_port = htons(PORT);

	return socket_data;
}

std::string send_message(std::string message, socketData socket_data){
	char buffer[1024];
	
	connect(socket_data.socket, (struct sockaddr*)&socket_data.server_address, sizeof(socket_data.server_address));
	send(socket_data.socket, (char*)message.c_str() , strlen((char*)message.c_str()) , 0 );
	int number_of_bytes_read = read(socket_data.socket, buffer, 1024);

	buffer[number_of_bytes_read] = '\0';
	std::string response = "";
	response.append(buffer);
	return response;
}