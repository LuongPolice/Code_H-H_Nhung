/* app.c - Điều khiển 3 driver kernel: mq2adc, dht11_driver, buzzer
 * Tích hợp watchdog, ghi log hệ thống, sử dụng cJSON
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <cjson/cJSON.h>

#define MQ2_PATH "/dev/mq2adc"
#define DHT11_PATH "/dev/dht11"
#define BUZZER_PATH "/dev/buzzer"
#define WATCHDOG_PATH "/dev/watchdog"
#define LOG_PATH "/var/log/sensor_monitor.log"
#define GAS_THRESHOLD 7000
#define LOOP_DELAY 2 // giây

int watchdog_fd = -1;

/* Ghi log văn bản kèm thời gian */
void log_text(const char *msg) {
    FILE *log = fopen(LOG_PATH, "a");
    if (log) {
        time_t now = time(NULL);
        char *ts = ctime(&now);
        ts[strlen(ts) - 1] = '\0'; // Xóa ký tự xuống dòng
        fprintf(log, "[%s] %s\n", ts, msg);
        fclose(log);
    }
}

/* Ghi log dữ liệu JSON */
void log_json(const char *type, int value) {
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type", type);
    cJSON_AddNumberToObject(obj, "value", value);
    cJSON_AddNumberToObject(obj, "timestamp", time(NULL));

    char *out = cJSON_PrintUnformatted(obj);
    if (out) {
        FILE *log = fopen(LOG_PATH, "a");
        if (log) {
            fprintf(log, "%s\n", out);
            fclose(log);
        }
        free(out);
    }
    cJSON_Delete(obj);
}

/* Đọc giá trị từ driver MQ2 (ADC) */
int read_mq2() {
    int fd = open(MQ2_PATH, O_RDONLY);
    if (fd < 0) return -1;
    char buf[32];
    int val = -1;
    if (read(fd, buf, sizeof(buf)) > 0) {
        sscanf(buf, "ADC: %d", &val);
    }
    close(fd);
    return val;
}

/* Đọc nhiệt độ và độ ẩm từ DHT11 (định dạng: Nhiet do: 30°C\nDo am: 57%) */
int read_dht11(int *temp, int *hum) {
    int fd = open(DHT11_PATH, O_RDONLY);
    if (fd < 0) return -1;
    char buf[64];
    if (read(fd, buf, sizeof(buf)) > 0) {
        sscanf(buf, "Nhiet do: %d°C\nDo am: %d%%", temp, hum);
        close(fd);
        return 0;
    }
    close(fd);
    return -1;
}

/* Bật hoặc tắt còi (1: bật, 0: tắt) */
int control_buzzer(int on) {
    int fd = open(BUZZER_PATH, O_WRONLY);
    if (fd < 0) return -1;
    if (on)
        write(fd, "ON", 2);
    else
        write(fd, "OFF", 3);
    close(fd);
    return 0;
}

/* Khởi tạo watchdog timer */
int init_watchdog() {
    watchdog_fd = open(WATCHDOG_PATH, O_RDWR);
    if (watchdog_fd < 0) return -1;
    int timeout = 60;
    ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &timeout);
    return 0;
}

/* Gửi tín hiệu giữ watchdog hoạt động */
void keep_alive_watchdog() {
    if (watchdog_fd >= 0)
        ioctl(watchdog_fd, WDIOC_KEEPALIVE, 0);
}

/* Vô hiệu hóa watchdog trước khi thoát */
void disable_watchdog() {
    if (watchdog_fd >= 0) {
        write(watchdog_fd, "V", 1); // magic character để tắt
        close(watchdog_fd);
    }
}

int main() {
    int last_buzzer = 0;
    init_watchdog();
    log_text("System started");

    while (1) {
        int gas = read_mq2();
        int temp = 0, hum = 0;
        read_dht11(&temp, &hum);

        time_t now = time(NULL);
        char *timestr = ctime(&now);
        timestr[strlen(timestr)-1] = '\0'; // bỏ \n

        printf("[%s] Gas: %d, Temp: %d°C, Humidity: %d%%\n", timestr, gas, temp, hum);

        char logbuf[128];
        snprintf(logbuf, sizeof(logbuf), "Gas: %d, Temp: %d, Humidity: %d", gas, temp, hum);
        log_text(logbuf);

        log_json("gas", gas);
        log_json("temperature", temp);
        log_json("humidity", hum);

        if (gas >= GAS_THRESHOLD) {
            if (!last_buzzer) {
                control_buzzer(1);
                log_text("Buzzer ON: Gas level exceeded threshold");
                printf("[%s] GAS: %d, Temp: %d°C, Hum: %d%% - Buzzer ON\n", timestr, gas, temp, hum);
                last_buzzer = 1;
            }
        } else {
            if (last_buzzer) {
                control_buzzer(0);
                log_text("Buzzer OFF: Gas level normal");
                printf("[%s] GAS: %d, Temp: %d°C, Hum: %d%% - Buzzer OFF\n", timestr, gas, temp, hum);
                last_buzzer = 0;
            }
        }

        keep_alive_watchdog();
        sleep(LOOP_DELAY);
    }

    disable_watchdog();
    log_text("System stopped");
    return 0;
}

