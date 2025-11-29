// udp_server.c
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

#define UDP_PORT        12345
#define MAX_PACKET_SIZE 1500

static pthread_t server_thread;
static int sockfd = -1;
static int running = 0;
static volatile int g_stopRequested = 0;

// ----------------------------------------------------
// Sensor mode API from main.c
// ----------------------------------------------------

// Must match the enum in main.c
typedef enum {
    SENSOR_MODE_HALL_ONLY = 0,
    SENSOR_MODE_AUTO      = 1,
    SENSOR_MODE_BEMF_ONLY = 2
} SensorMode_t;

// Provided by main.c
extern void         Control_setSensorMode(SensorMode_t mode);
extern SensorMode_t Control_getSensorMode(void);

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

// ----------------------------------------------------
// Small helpers to stringify enums
// ----------------------------------------------------
static const char* motor_state_to_str(MotorState_t s)
{
    switch (s) {
    case MOTOR_STATE_IDLE:  return "IDLE";
    case MOTOR_STATE_ALIGN: return "ALIGN";
    case MOTOR_STATE_RUN:   return "RUN";
    case MOTOR_STATE_FAULT: return "FAULT";
    default:                return "UNKNOWN";
    }
}

static const char* motor_fault_to_str(MotorFault_t f)
{
    switch (f) {
    case MOTOR_FAULT_NONE:        return "NONE";
    case MOTOR_FAULT_OVERCURRENT: return "OVERCURRENT";
    case MOTOR_FAULT_OVERVOLT:    return "OVERVOLT";
    case MOTOR_FAULT_UNDERVOLT:   return "UNDERVOLT";
    case MOTOR_FAULT_HALL_TIMEOUT:return "HALL_TIMEOUT";
    case MOTOR_FAULT_DRV8302:     return "DRV8302";
    case MOTOR_FAULT_TIMING:      return "TIMING";
    default:                      return "UNKNOWN";
    }
}

static const char* sensor_mode_to_str(SensorMode_t m)
{
    switch (m) {
    case SENSOR_MODE_HALL_ONLY: return "HALL_ONLY";
    case SENSOR_MODE_AUTO:      return "AUTO";
    case SENSOR_MODE_BEMF_ONLY: return "BEMF_ONLY";
    default:                    return "UNKNOWN";
    }
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
        "  enable                    -- enable motor\n"
        "  disable                   -- disable motor\n"
        "  set rpm <value>           -- set speed command (0-5000)\n"
        "  set dir <fwd|rev>         -- set direction\n"
        "  sensor                    -- get sensor mode (0=hall,1=auto,2=bemf)\n"
        "  sensor <hall|auto|bemf>   -- set sensor mode\n"
        "  status                    -- get motor state & telemetry\n"
        "  stop                      -- shutdown program\n"
        "  help                      -- show this help\n";
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
        MotorControl_setSpeedCmd((float)rpm,
                                 MotorControl_getContext().cmd.direction);
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
        // direction bool: 0 = fwd, 1 = rev
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
        else if (strcmp(tok, "sensor") == 0 || strcmp(tok, "sens") == 0) {
            char *arg1 = strtok(NULL, " \t");
            if (!arg1) {
                // QUERY: sensor -> return current mode as number
                SensorMode_t m = Control_getSensorMode();
                char msg[32];
                snprintf(msg, sizeof(msg), "%d\n", (int)m);  // 0,1,2
                send_response(msg, &client_addr, addr_len);
            } else {
                SensorMode_t mode;
                if (strcmp(arg1, "hall") == 0) {
                    mode = SENSOR_MODE_HALL_ONLY;
                } else if (strcmp(arg1, "auto") == 0) {
                    mode = SENSOR_MODE_AUTO;
                } else if (strcmp(arg1, "bemf") == 0 ||
                           strcmp(arg1, "sensorless") == 0) {
                    mode = SENSOR_MODE_BEMF_ONLY;
                } else {
                    send_response("ERR: sensor mode must be hall|auto|bemf.\n",
                                  &client_addr, addr_len);
                    continue;
                }

                Control_setSensorMode(mode);

                char msg[64];
                snprintf(msg, sizeof(msg),
                         "OK: sensor mode set to %d(%s)\n",
                         (int)mode, sensor_mode_to_str(mode));
                send_response(msg, &client_addr, addr_len);
            }
        }
        else if (strcmp(tok, "status") == 0) {
            MotorContext_t ctx = MotorControl_getContext();
            PosEst_t pe = PosEst_get();
            SensorMode_t sm = Control_getSensorMode();

            char msg[256];
            snprintf(msg, sizeof(msg),
                     "STATE=%d(%s) FAULT=%d(%s) "
                     "RPM=%.1f CMD=%.1f DUTY=%.3f "
                     "SECTOR=%u DIR=%d SENSOR_MODE=%d(%s)\n",
                     ctx.state,
                     motor_state_to_str(ctx.state),
                     ctx.fault,
                     motor_fault_to_str(ctx.fault),
                     ctx.meas.rpm_mech,
                     ctx.cmd.rpm_cmd,
                     ctx.cmd.torque_cmd,
                     pe.sector,
                     ctx.cmd.direction,
                     (int)sm,
                     sensor_mode_to_str(sm));
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