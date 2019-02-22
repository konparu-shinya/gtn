/********************************************************************************
 * gtnのAction実行部
 ********************************************************************************/
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define QUEUELIMIT 5

int main(void)
{
	int servSock;				//server socket descripter
	int clitSock;				//client socket descripter
	int on;
	struct sockaddr_in servSockAddr;	//server internet socket address
	struct sockaddr_in clitSockAddr;	//client internet socket address
	unsigned short servPort = 9001;		//server port number
	unsigned int clitLen;			// client internet socket address length

	if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 ){
		perror("socket() failed.");
		exit(EXIT_FAILURE);
	}
	/* Enable address reuse */
	on = 1;
	setsockopt (servSock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
 
	memset(&servSockAddr, 0, sizeof(servSockAddr));
	servSockAddr.sin_family      = AF_INET;
	servSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servSockAddr.sin_port        = htons(servPort);

	if (bind(servSock, (struct sockaddr *) &servSockAddr, sizeof(servSockAddr) ) < 0 ) {
		perror("bind() failed.");
		exit(EXIT_FAILURE);
	}

	if (listen(servSock, QUEUELIMIT) < 0) {
		perror("listen() failed.");
		exit(EXIT_FAILURE);
	}

	while(1) {
		clitLen = sizeof(clitSockAddr);
		if ((clitSock = accept(servSock, (struct sockaddr *) &clitSockAddr, &clitLen)) < 0) {
			perror("accept() failed.");
			exit(EXIT_FAILURE);
		}

		printf("connected from %s.\n", inet_ntoa(clitSockAddr.sin_addr));
		close(clitSock);
	}
	return 0;
}


