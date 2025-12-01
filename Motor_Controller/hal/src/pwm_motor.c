// hal/src/pwm_motor.c
#include "pwm_motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_PWM_FREQ_HZ   20000.0   // 20 kHz, adjust if you like
#define PERIOD_NS_MAX       469754879ULL   // from your single‑channel driver
#define PERIOD_NS_MIN       1000ULL

// ---------------- low-level helpers ----------------

static int write_str_exact(const char *path, const char *s)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return -1;
    }

    size_t slen = strlen(s);
    const char *out = s;
    size_t outlen = slen;
    char buf[64];

    // ensure newline
    if (slen == 0 || s[slen - 1] != '\n') {
        int n = snprintf(buf, sizeof(buf), "%s\n", s);
        if (n <= 0 || (size_t)n >= sizeof(buf)) {
            fprintf(stderr, "snprintf(%s) overflow\n", path);
            close(fd);
            return -1;
        }
        out = buf;
        outlen = (size_t)n;
    }

    ssize_t w = write(fd, out, outlen);
    int saved = errno;
    if (close(fd) != 0 && w >= 0) {
        errno = saved;
        fprintf(stderr, "close %s: %s\n", path, strerror(errno));
        return -1;
    }
    if (w < 0 || (size_t)w != outlen) {
        errno = saved;
        fprintf(stderr, "write %s: %s\n", path, strerror(errno));
        return -1;
    }
    return 0;
}

static int write_u64(const char *path, unsigned long long v)
{
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%llu", v);
    if (n <= 0 || (size_t)n >= sizeof(tmp)) {
        fprintf(stderr, "snprintf overflow for %s\n", path);
        return -1;
    }
    return write_str_exact(path, tmp);
}

static int read_first_char(const char *path, char *out)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return -1;
    }
    char c = 0;
    ssize_t r = read(fd, &c, 1);
    int saved = errno;
    close(fd);
    if (r < 0) {
        errno = saved;
        fprintf(stderr, "read %s: %s\n", path, strerror(errno));
        return -1;
    }
    *out = c;
    return 0;
}

/* enable: 1/0; tolerate redundant writes that return EINVAL */
static int pwm_set_enabled(const char *enable_path, int on)
{
    char c = 0;
    if (read_first_char(enable_path, &c) == 0) {
        int cur = (c == '1');
        if (cur == !!on) return 0;   // already desired state
    }
    if (write_str_exact(enable_path, on ? "1" : "0") == 0) return 0;
    if (errno == EINVAL) return 0;
    return -1;
}

// Configure fixed frequency for one channel and set duty
static int pwm_config_channel(PwmMotor_t *m,
                              PwmMotorChannel_t *ch,
                              unsigned long long period_ns,
                              float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    unsigned long long duty_ns =
        (unsigned long long)((double)period_ns * (double)duty);

    if (duty_ns >= period_ns) duty_ns = period_ns - 1;
    if (duty_ns == 0 && duty > 0.0f) duty_ns = 1;

    // always start disabled when changing period
    (void)pwm_set_enabled(ch->enable_path, 0);

    // STRATEGY A: duty=0 (or 1) -> period -> desired duty -> enable
    if (write_u64(ch->duty_path, 0) != 0) {
        if (errno != EINVAL) return -1;
        if (write_u64(ch->duty_path, 1) != 0) return -1;
    }
    if (write_u64(ch->period_path, period_ns) == 0 &&
        write_u64(ch->duty_path, duty_ns) == 0 &&
        pwm_set_enabled(ch->enable_path, 1) == 0) {
        return 0;
    }

    // STRATEGY B
    (void)pwm_set_enabled(ch->enable_path, 0);
    if (write_u64(ch->period_path, period_ns) == 0 &&
        write_u64(ch->duty_path, duty_ns) == 0 &&
        pwm_set_enabled(ch->enable_path, 1) == 0) {
        return 0;
    }

    // STRATEGY C
    (void)pwm_set_enabled(ch->enable_path, 0);
    if (write_u64(ch->duty_path, 1) == 0 &&
        write_u64(ch->period_path, period_ns) == 0 &&
        write_u64(ch->duty_path, duty_ns) == 0 &&
        pwm_set_enabled(ch->enable_path, 1) == 0) {
        return 0;
    }

    return -1;
}

// Just update duty (period already configured)
static int pwm_set_duty(PwmMotorChannel_t *ch,
                        unsigned long long period_ns,
                        float duty)
{
    if (duty <= 0.0f) {
        return write_u64(ch->duty_path, 0);
    }
    if (duty > 1.0f) duty = 1.0f;

    unsigned long long duty_ns =
        (unsigned long long)((double)period_ns * (double)duty);

    if (duty_ns >= period_ns) duty_ns = period_ns - 1;
    if (duty_ns == 0) duty_ns = 1;

    return write_u64(ch->duty_path, duty_ns);
}

// ---------------- PwmMotor API ----------------

static int build_channel_paths(PwmMotorChannel_t *ch,
                               const char *root,
                               unsigned int gpio_num)
{
    // base = "<root>/GPIO<nn>"
    char base[128];
    int n = snprintf(base, sizeof(base), "%s/GPIO%u", root, gpio_num);
    if (n <= 0 || (size_t)n >= sizeof(base)) {
        fprintf(stderr, "pwm path overflow\n");
        return -1;
    }

    n = snprintf(ch->period_path, sizeof(ch->period_path),
                 "%s/period", base);
    if (n <= 0 || (size_t)n >= sizeof(ch->period_path)) return -1;

    n = snprintf(ch->duty_path, sizeof(ch->duty_path),
                 "%s/duty_cycle", base);
    if (n <= 0 || (size_t)n >= sizeof(ch->duty_path)) return -1;

    n = snprintf(ch->enable_path, sizeof(ch->enable_path),
                 "%s/enable", base);
    if (n <= 0 || (size_t)n >= sizeof(ch->enable_path)) return -1;

    // quick existence check
    if (access(ch->period_path, R_OK | W_OK) != 0 ||
        access(ch->duty_path,   R_OK | W_OK) != 0 ||
        access(ch->enable_path, R_OK | W_OK) != 0) {
        fprintf(stderr,
                "PWM channel for GPIO%u not available under %s\n",
                gpio_num, root);
        return -1;
    }
    return 0;
}

bool PwmMotor_init(PwmMotor_t *m,
                   const char *pwm_root,
                   unsigned int inh_a_gpio,
                   unsigned int inl_a_gpio,
                   unsigned int inh_b_gpio,
                   unsigned int inl_b_gpio,
                   unsigned int inh_c_gpio,
                   unsigned int inl_c_gpio)
{
    if (!m || !pwm_root) return false;
    memset(m, 0, sizeof(*m));

    if (build_channel_paths(&m->ch[0], pwm_root, inh_a_gpio) != 0) return false;
    if (build_channel_paths(&m->ch[1], pwm_root, inl_a_gpio) != 0) return false;
    if (build_channel_paths(&m->ch[2], pwm_root, inh_b_gpio) != 0) return false;
    if (build_channel_paths(&m->ch[3], pwm_root, inl_b_gpio) != 0) return false;
    if (build_channel_paths(&m->ch[4], pwm_root, inh_c_gpio) != 0) return false;
    if (build_channel_paths(&m->ch[5], pwm_root, inl_c_gpio) != 0) return false;

    m->freq_hz = MOTOR_PWM_FREQ_HZ;

    unsigned long long period_ns =
        (unsigned long long)(1000000000.0 / m->freq_hz);
    if (period_ns == 0) period_ns = 1;
    if (period_ns > PERIOD_NS_MAX) period_ns = PERIOD_NS_MAX;
    if (period_ns < PERIOD_NS_MIN) period_ns = PERIOD_NS_MIN;
    m->period_ns = period_ns;

    // Configure all channels for this freq with duty=0
    for (int i = 0; i < 6; ++i) {
        if (pwm_config_channel(m, &m->ch[i], period_ns, 0.0f) != 0) {
            fprintf(stderr, "Failed to configure PWM motor channel %d\n", i);
            return false;
        }
    }

    m->enabled = true;
    return true;
}

void PwmMotor_setEnable(PwmMotor_t *m, bool enable)
{
    if (!m) return;

    m->enabled = enable;
    for (int i = 0; i < 6; ++i) {
        if (!enable) {
            // duty 0 and disable
            (void)pwm_set_duty(&m->ch[i], m->period_ns, 0.0f);
            (void)pwm_set_enabled(m->ch[i].enable_path, 0);
        } else {
            (void)pwm_set_enabled(m->ch[i].enable_path, 1);
        }
    }
}

void PwmMotor_applyPhaseState(PwmMotor_t *m,
                              int u, int v, int w,
                              float duty)
{
    if (!m) return;
    if (!m->enabled) {
        // keep everything off
        for (int i = 0; i < 6; ++i) {
            (void)pwm_set_duty(&m->ch[i], m->period_ns, 0.0f);
        }
        return;
    }

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // phase A -> ch[0] = INH-A, ch[1] = INL-A
    float duty_inh_a = 0.0f, duty_inl_a = 0.0f;
    if (u > 0) duty_inh_a = duty;
    else if (u < 0) duty_inl_a = duty;

    float duty_inh_b = 0.0f, duty_inl_b = 0.0f;
    if (v > 0) duty_inh_b = duty;
    else if (v < 0) duty_inl_b = duty;

    float duty_inh_c = 0.0f, duty_inl_c = 0.0f;
    if (w > 0) duty_inh_c = duty;
    else if (w < 0) duty_inl_c = duty;

    (void)pwm_set_duty(&m->ch[0], m->period_ns, duty_inh_a);
    (void)pwm_set_duty(&m->ch[1], m->period_ns, duty_inl_a);
    (void)pwm_set_duty(&m->ch[2], m->period_ns, duty_inh_b);
    (void)pwm_set_duty(&m->ch[3], m->period_ns, duty_inl_b);
    (void)pwm_set_duty(&m->ch[4], m->period_ns, duty_inh_c);
    (void)pwm_set_duty(&m->ch[5], m->period_ns, duty_inl_c);
}

// same sector_to_signs as before
static void sector_to_signs(uint8_t sector,
                            bool forward,
                            int *u, int *v, int *w)
{
    *u = *v = *w = 0;
    if (sector > 5) return;

    if (forward) {
        switch (sector) {
        case 0: *u = +1; *v = -1; *w =  0; break;
        case 1: *u = +1; *v =  0; *w = -1; break;
        case 2: *u =  0; *v = +1; *w = -1; break;
        case 3: *u = -1; *v = +1; *w =  0; break;
        case 4: *u = -1; *v =  0; *w = +1; break;
        case 5: *u =  0; *v = -1; *w = +1; break;
        }
    } else {
        switch (sector) {
        case 0: *u = -1; *v = +1; *w =  0; break;
        case 1: *u = -1; *v =  0; *w = +1; break;
        case 2: *u =  0; *v = -1; *w = +1; break;
        case 3: *u = +1; *v = -1; *w =  0; break;
        case 4: *u = +1; *v =  0; *w = -1; break;
        case 5: *u =  0; *v = +1; *w = -1; break;
        }
    }
}

void PwmMotor_setSixStep(PwmMotor_t *m,
                         uint8_t sector,
                         float duty,
                         bool forward)
{
    if (!m) return;
    int u = 0, v = 0, w = 0;
    sector_to_signs(sector, forward, &u, &v, &w);
    PwmMotor_applyPhaseState(m, u, v, w, duty);
}

void PwmMotor_stop(PwmMotor_t *m)
{
    if (!m) return;
    PwmMotor_setEnable(m, false);
}

void PwmMotor_deinit(PwmMotor_t *m)
{
    if (!m) return;
    PwmMotor_stop(m);
    // nothing else to close; /dev nodes are global
}
