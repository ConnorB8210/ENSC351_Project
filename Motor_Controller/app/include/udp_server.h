// udp_server.h
#pragma once

#include <stdbool.h>

// Start the UDP server in its own thread.
// Returns true on success, false on error.
bool UDPServer_init(void);

// Stop the UDP server thread and close the socket.
void UDPServer_cleanup(void);

// Returns non‑zero if a remote client has sent "stop".
int UDPServer_wasStopRequested(void);