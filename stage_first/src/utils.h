#ifndef _utils_h_
#define _utils_h_

#include <iostream>
#include <vector>

#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9))
#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

void printArrayI(const char *name, const int32_t *data, uint32_t size);
void printArrayF(const char *name, const double *data, uint32_t size);

int32_t thread_rt();
int32_t process_rt();
int32_t sched_thread(int p = 40);
int32_t sched_process(int p = 40);

bool readCsvData(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data);

int kbhit(void);

void print_cpu_mask(cpu_set_t cpu_mask);
int8_t get_cpu_mask(pid_t pid, cpu_set_t *mask);
int8_t set_cpu_mask(pid_t pid, cpu_set_t *mask);

#endif
