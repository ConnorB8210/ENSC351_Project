// udp_server_motor.c
#include "udp_server.h"
#include "motor_control.h"
#include "motor_states.h"
#include "position_estimator.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <time.h>      // <-- added

#define UDP_PORT        12345
#define MAX_PACKET_SIZE 1500

static pthread_t server_thread;
static int sockfd = -1;
static int running = 0;
static volatile int g_stopRequested = 0;

static void* udp_thread_func(void* arg);
static void send_response(const char* response,
                          struct sockaddr_in* client_addr,
                          socklen_t addr_len);

static void to_lower_str(char *s)
{
    for (; *s; ++s) {
        *s = (char)tolower((unsigned char)*s);
    }
}

// simple local time helper (seconds, monotonic-ish)
static double get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// ----------------------------------------------------
// PUBLIC API
// ----------------------------------------------------
bool UDPServer_init(void)
{
    struct sockaddr_in server_addr;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return false;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(UDP_PORT);

    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        close(sockfd);
        sockfd = -1;
        return false;
    }

    running = 1;
    g_stopRequested = 0;

    if (pthread_create(&server_thread, NULL, udp_thread_func, NULL) != 0) {
        perror("pthread_create");
        close(sockfd);
        sockfd = -1;
        running = 0;
        return false;
    }

    printf("Motor UDP server started on port %d.\n", UDP_PORT);
    return true;
}

void UDPServer_cleanup(void)
{
    if (!running && sockfd < 0) return;

    running = 0;
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
    pthread_join(server_thread, NULL);
    printf("Motor UDP server stopped.\n");
}

int UDPServer_wasStopRequested(void)
{
    return g_stopRequested;
}

// ----------------------------------------------------
// COMMANDS
// ----------------------------------------------------
static void print_help(struct sockaddr_in* client_addr, socklen_t addr_len)
{
    const char *msg =
        "Motor Control UDP Commands:\n"
        "  enable               -- enable motor\n"
        "  disable              -- disable motor\n"
        "  set rpm <value>      -- set speed command (0-5000)\n"
        "  set dir <fwd|rev>    -- set direction\n"
        "  status               -- get motor state & telemetry\n"
        "  statusraw            -- CSV: t,rpm_cmd,rpm_mech,torque,vbus,state,fault\n"
        "  stop                 -- shutdown program\n"
        "  help                 -- show this help\n";
    send_response(msg, client_addr, addr_len);
}

static void handle_set(struct sockaddr_in* client_addr,
                       socklen_t addr_len,
                       char *arg1)
{
    if (!arg1) {
        send_response("ERR: set <rpm|dir> ...\n", client_addr, addr_len);
        return;
    }

    // SET RPM ---------------------------
    if (strcmp(arg1, "rpm") == 0) {
        char *arg2 = strtok(NULL, " \t");
        if (!arg2) {
            send_response("ERR: set rpm <value>\n", client_addr, addr_len);
            return;
        }
        long rpm = strtol(arg2, NULL, 10);
        if (rpm < 0 || rpm > MOTOR_RPM_MAX) {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "ERR: rpm must be 0-%d.\n", (int)MOTOR_RPM_MAX);
            send_response(msg, client_addr, addr_len);
            return;
        }
        MotorControl_setSpeedCmd((float)rpm, MotorControl_getContext().cmd.direction);
        send_response("OK: rpm updated\n", client_addr, addr_len);
        return;
    }

    // SET DIRECTION ---------------------
    if (strcmp(arg1, "dir") == 0) {
        char *arg2 = strtok(NULL, " \t");
        if (!arg2) {
            send_response("ERR: set dir <fwd|rev>\n", client_addr, addr_len);
            return;
        }
        bool forward = true;
        if (strcmp(arg2, "fwd") == 0 || strcmp(arg2, "forward") == 0) {
            forward = true;
        }
        else if (strcmp(arg2, "rev") == 0 || strcmp(arg2, "reverse") == 0) {
            forward = false;
        } else {
            send_response("ERR: direction must be fwd|rev\n", client_addr, addr_len);
            return;
        }

        MotorContext_t ctx = MotorControl_getContext();
        // direction: 0 = fwd, 1 = rev
        MotorControl_setSpeedCmd(ctx.cmd.rpm_cmd, !forward);
        send_response("OK: direction updated\n", client_addr, addr_len);
        return;
    }

    send_response("ERR: unknown set command\n", client_addr, addr_len);
}

// ----------------------------------------------------
// UDP THREAD
// ----------------------------------------------------
static void* udp_thread_func(void* arg)
{
    (void)arg;
    char buffer[MAX_PACKET_SIZE];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    while (running) {
        ssize_t bytes = recvfrom(sockfd, buffer, sizeof(buffer)-1, 0,
                                 (struct sockaddr*)&client_addr, &addr_len);
        if (bytes < 0) {
            if (running) perror("recvfrom");
            break;
        }

        buffer[bytes] = '\0';

        char work[MAX_PACKET_SIZE];
        strncpy(work, buffer, sizeof(work)-1);
        work[sizeof(work)-1] = '\0';
        to_lower_str(work);

        char *tok = strtok(work, " \t");
        if (!tok) {
            send_response("ERR: empty command\n", &client_addr, addr_len);
            continue;
        }

        // ------------------------------------------------
        // COMMANDS
        // ------------------------------------------------
        if (strcmp(tok, "help") == 0) {
            print_help(&client_addr, addr_len);
        }
        else if (strcmp(tok, "enable") == 0) {
            MotorControl_setEnable(true);
            send_response("OK: motor enabled\n", &client_addr, addr_len);
        }
        else if (strcmp(tok, "disable") == 0) {
            MotorControl_setEnable(false);
            send_response("OK: motor disabled\n", &client_addr, addr_len);
        }
        else if (strcmp(tok, "set") == 0) {
            char *arg1 = strtok(NULL, " \t");
            handle_set(&client_addr, addr_len, arg1);
        }
        else if (strcmp(tok, "status") == 0) {
            MotorContext_t ctx = MotorControl_getContext();
            PosEst_t pe = PosEst_get();
            char msg[256];
            snprintf(msg, sizeof(msg),
                     "STATE=%d FAULT=%d "
                     "RPM=%.1f CMD=%.1f DUTY=%.3f "
                     "SECTOR=%u DIR=%d VBUS=%.2f\n",
                     ctx.state,
                     ctx.fault,
                     ctx.meas.rpm_mech,
                     ctx.cmd.rpm_cmd,
                     ctx.cmd.torque_cmd,
                     pe.sector,
                     ctx.cmd.direction,
                     ctx.meas.v_bus);
            send_response(msg, &client_addr, addr_len);
        }
        else if (strcmp(tok, "statusraw") == 0) {
            // CSV log: t,rpm_cmd,rpm_mech,torque_cmd,v_bus,state,fault
            double t   = get_time_s();
            MotorContext_t ctx = MotorControl_getContext();

            char msg[256];
            snprintf(msg, sizeof(msg),
                     "%.6f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                     t,
                     ctx.cmd.rpm_cmd,      // internal slewed command
                     ctx.meas.rpm_mech,    // measured RPM
                     ctx.cmd.torque_cmd,   // PI output / duty
                     ctx.meas.v_bus,       // bus voltage
                     ctx.state,
                     ctx.fault);
            send_response(msg, &client_addr, addr_len);
        }
        else if (strcmp(tok, "stop") == 0) {
            send_response("OK: shutdown requested\n", &client_addr, addr_len);
            g_stopRequested = 1;
            running = 0;
            break;
        }
        else {
            send_response("ERR: unknown command. Type 'help'\n",
                          &client_addr, addr_len);
        }
    }

    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
    return NULL;
}

static void send_response(const char* response,
                          struct sockaddr_in* client_addr,
                          socklen_t addr_len)
{
    if (sockfd < 0) return;
    sendto(sockfd, response, strlen(response), 0,
           (struct sockaddr*)client_addr, addr_len);
}