#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT	 8080
#define MAXLINE 1024

int sockfd;
struct sockaddr_in	 servaddr;

void startMessaging(){
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	char buffer[MAXLINE];
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_addr.s_addr = inet_addr("192.168.1.21");
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
}
void stopMessaging(){
	close(sockfd);
}

void sendMessage(Tags tags){

	uint8_t deviation = tags.deviation*100;
	uint8_t speed = tags.speed*100;
	uint8_t status = 0;
	std::bitset<8> bits(status);
	bits[0] = tags.status;
	bits[1] = tags.cam_status;
	bits[2] = tags.drive_status;
	status = bits.to_ulong();

	char hello[] = { 'B', (char)tags.module_number, deviation, speed, status, 'E' };

	int n = sendto(sockfd, (const char *)hello, 6, MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
}