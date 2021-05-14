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

bool program1 = false;
bool program2 = false;
bool program3 = false;
void sendMessage(Tags tags){

	uint8_t deviation = abs(tags.deviation*100);
	uint8_t speed = tags.speed*100;
	uint8_t status = 0;
	std::bitset<8> bits(status);
	bits[0] = tags.status;
	bits[1] = tags.cam_status;
	bits[2] = tags.drive_status;
	bits[3] = tags.underspeed;
	bits[4] = (tags.deviation<0);
	bits[5] = program1;
	bits[6] = program2;
	bits[7] = program3;
	status = bits.to_ulong();

	char hello[] = { 'B', (char)tags.module_number, deviation, speed, status, 'E' };

	unsigned int len;

	int n = sendto(sockfd, (const char *)hello, 6, MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
	
	char buffer[100];
	int r = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)NULL, NULL);
	
	if (r != 3){
		//std::cout << "receive timeout, sent: " << n << std::endl;
		images.program = 1;
	}
	else{
		images.program = (int)buffer[1];
		program1 = 0;
		program2 = 0;
		program3 = 0;
		if (images.program == 1)
			program1 = 1;
		if (images.program == 2)
			program2 = 1;
		if (images.program == 3)
			program3 = 1;
	}
	
	//std::cout << "selected program: " << (int)buffer[1] << "\t";// << std::endl;
}
