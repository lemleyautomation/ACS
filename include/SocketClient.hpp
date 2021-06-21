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

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 3000;
	if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
		perror("Error");
	}
}
void stopMessaging(){
	close(sockfd);
}

void sendMessage(Tags tags){

	uint8_t deviation = abs(tags.deviation*100);
	uint8_t speed = tags.speed*100;
	uint8_t status = 0;
	std::bitset<8> bits(status);
	bits[0] = tags.status;
	bits[1] = tags.cam_status;
	bits[2] = tags.drive_status;
	bits[3] = tags.underspeed;
	bits[4] = 0;
	bits[5] = (images.program==1);
	bits[6] = (images.program==2);
	bits[7] = (images.program==3);
	status = bits.to_ulong();

	char hello[] = { 'B', (char)tags.module_number, deviation, speed, status, 'E' };

	unsigned int len;
	int n = sendto(sockfd, (const char *)hello, 6, MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
	
	char buffer[100];
	int r = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)NULL, NULL);
	
	//std::cout << "recieved message of " << r << " bytes: " << buffer[0] << " " << int(buffer[1]) << " " << (int)buffer[2] << std::endl;

	if (r <= 1){
		images.program = 1;
	}
	else{
		images.program = (int)buffer[1];
		images.trim = (int)buffer[2];
	}
}
