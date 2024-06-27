// Here we track cpu and ram usage and log it in MonitorData. Not that easy to get that data on Linux!

#include "statistics.h"
#include "main.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <stdio.h>

struct CpuCapture
{
    // Absolute values since last reboot.
    unsigned long long idletime;
    unsigned long long workingtime;
};

CpuCapture prev_capture[4];
float cpu_timer = 1;

struct CpuStat
{
    char name[20];
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
};

struct CpuCapture capture_from_cpustat(const CpuStat& s)
{
    CpuCapture r;
    r.idletime = s.idle + s.iowait;
    r.workingtime = s.user + s.nice + s.system + s.irq + s.softirq;
    return r;
}

float usage_from_capture(const CpuCapture& now, const CpuCapture& prev)
{
    // the number of ticks that passed by since the last measurement
    const unsigned long long workingtime = now.workingtime - prev.workingtime;
    const unsigned long long alltime = workingtime + (now.idletime - prev.idletime);
    // they are divided by themselves - so the unit does not matter.
    return (float)((double)workingtime / alltime);
}

void get_cpu_usage()
{
    // Code is slightly modified from here: https://stackoverflow.com/a/69246433
    md.cpu_usage[0] = -1;
    md.cpu_usage[1] = -1;
    md.cpu_usage[2] = -1;
    md.cpu_usage[3] = -1;
    int stat = open("/proc/stat", O_RDONLY);
    if (stat == -1)
    {
		perror("/proc/stat");
        return;
    }
    // let's read everything in one call so it's nicely synced.
    char buffer[10001];
    const ssize_t readed = read(stat, buffer, sizeof(buffer) - 1);
    assert(readed != -1);
    buffer[readed] = '\0';
    close(stat);
    // Read the values from the read buffer
    FILE *f = fmemopen(buffer, readed, "r");

	CpuStat c = {0};
    while (fscanf(f, "%19s %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", c.name, &c.user, &c.nice,
            &c.system, &c.idle, &c.iowait, &c.irq, &c.softirq, &c.steal, &c.guest,
            &c.guest_nice) == 11)
    {
        if (c.name[0] == 'c' &&
            c.name[1] == 'p' &&
            c.name[2] == 'u' &&
            c.name[3] >= '0' &&
            c.name[3] <= '3')
        {
            CpuCapture now = capture_from_cpustat(c);
            int i = c.name[3] - '0';
            assert(i >= 0 && i < 4);
            CpuCapture& prev = prev_capture[i];
            md.cpu_usage[i] = usage_from_capture(now, prev);
            prev = now;
        }
    }
    fclose(f);
}

void get_ram_usage()
{
    md.ram_avail_kb = -1;
    md.swap_avail_kb = -1;
    struct sysinfo info;
    if (sysinfo(&info) != 0)
    {
        perror("sysinfo");
        return;
    }
    md.swap_avail_kb = info.freeswap/1024;

    /*printf("Total RAM: %ld KB\n", info.totalram/1024);
    printf("Free RAM: %ld KB\n", info.freeram/1024);
    printf("Free Swap: %ld KB\n", info.freeswap/1024);
    printf("buffered: %ld KB\n", info.bufferram/1024);
    printf("mem_unit: %d\n", info.mem_unit);*/

    int stat = open("/proc/meminfo", O_RDONLY);
    if (stat == -1)
    {
		perror("/proc/meminfo");
        return;
    }
    char buffer[10001];
    const ssize_t readed = read(stat, buffer, sizeof(buffer) - 1);
    assert(readed != -1);
    buffer[readed] = '\0';
    close(stat);
    // Read the values from the read buffer
    FILE *f = fmemopen(buffer, readed, "r");

    while (true)
    {
        char name[20];
        char name2[4];
        unsigned long long value;
        int r = fscanf(f, "%19s%llu%4s", name, &value, name2);
        if (r != 3)
            break;
        if (strcmp(name, "MemAvailable:") == 0)
        {
            //printf("available: %ld KB\n", value);
            md.ram_avail_kb = (int)value;
            break;
        }
    }
    fclose(f);
}

bool statistics_update()
{
    cpu_timer += md.delta_time;
    if (cpu_timer < 0.5f) return true;
    cpu_timer = 0;

    get_cpu_usage();
    get_ram_usage();
	return true;
}

