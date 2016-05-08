#include <iostream>
#include <stdio.h>
#include <nn.h>
#include <pair.h>
#include <windows.h>// for sleep

using namespace std;

void node_client() {
	int sock = nn_socket(AF_SP, NN_PAIR);
	cout << (sock > 0) << endl;
	if (nn_connect(sock, "ipc://test") >= 0) {
		cout << "Connected to server\n";
	}
	else {
		cout << "Failed to connect to server\n";
	}
	while (true) {
		char *buf = NULL;
		int val = 0;
		char name[128];
		int bytes = nn_recv(sock, &buf, NN_MSG, 0);
		if (bytes >= 0) {
			printf("CLIENT RECEIVED %d bytes: ", bytes);
			sscanf(buf, "%4s,%d", name, &val);
			printf("msg_name = %s, val = %d\n", name, val);
			nn_freemsg(buf);
		}
	}
}

void node_server() {
	int sock = nn_socket(AF_SP, NN_PAIR);
	cout << (sock > 0) << endl;
	if (nn_bind(sock, "ipc://test") >= 0) {
		cout << "Server bound\n";
	}
	else {
		cout << "Failed to bind socket on server\n";
	}
	int counter = 0;
	char buffer[256];
	while (true) {
		counter++;
		sprintf_s(buffer, "%s,%d", "mode",counter);
		cout << "SERVER SENDING " << buffer;
		int bytes = nn_send(sock, buffer, 4+sizeof(counter), NN_DONTWAIT);
		if (bytes > 0) {
			cout << "=" << bytes << "bytes\n";
		}
		else {
			cout << "Could not send\n";
		}
		Sleep(1000);
	}
}

int main(int argc, char** argv) {

	if (argc < 2) {
		cout << "USAGE: \"nanomsg_test.exe SERVER/CLIENT\" \n\t Use 0 for Server and 1 for Client\n\t e.g. \"nanomsg_test.exe 1\"\n ";
	}
	else {
		int input = atoi(argv[1]);
		switch (input) {
		case 0:
			cout << "SERVER\n";
			node_server();
			break;
		case 1:
			cout << "CLIENT \n";
			node_client();
			break;
		}
	}

	return 0;
}