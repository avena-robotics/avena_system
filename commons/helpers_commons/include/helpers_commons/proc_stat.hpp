/*
Copyright (c) 2017, Adam Howard
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
     
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PROC_STAT_H
#define PROC_STAT_H

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <fstream>

typedef struct proc_stat_t
{
  int pid;
  char *comm, state;
  int ppid, pgrp, session, tty_nr, tpgid;
  unsigned int flags;
  unsigned long int minflt, cminflt, majflt, cmajflt, utime, stime;
  long int cutime, cstime, priority, nice, num_threads, itrealvalue;
  unsigned long long int starttime;
  unsigned long int vsize;
  long int rss;
  unsigned long int rsslim;
  void *startcode, *endcode, *startstack, *kstkesp, *kstkeip;
  unsigned long int signal, blocked, sigignore, sigcatch;
  void *wchan;
  unsigned long int nswap, cnswap;
  int exit_signal, processor;
  unsigned int rt_priority, policy;
  unsigned long long int delayacct_blkio_ticks;
  unsigned long int guest_time;
  long int cguest_time;
  void *start_data, *end_data, *start_brk, *arg_start, *arg_end;
  void *env_start, *env_end;
  int exit_code;
} ProcStat;

typedef struct uptime_t
{
  float uptime, idle;
} Uptime;

static inline uint32_t get_cpu_jiffies()
{
  std::ifstream proc_stat("/proc/stat");
  std::string cpu_jiffies_str;
  std::getline(proc_stat, cpu_jiffies_str);
  cpu_jiffies_str = cpu_jiffies_str.substr(3, std::string::npos);
  std::stringstream ss(cpu_jiffies_str);
  uint32_t cpu_jiffies;
  uint32_t current_jiffy;
  while (ss >> current_jiffy)
  {
    cpu_jiffies += current_jiffy;
  }
  return cpu_jiffies;
}
static inline uint32_t get_total_cpu_jiffies()
{
  std::ifstream proc_stat("/proc/stat");
  std::string cpu_jiffies_str;
  std::getline(proc_stat, cpu_jiffies_str);
  cpu_jiffies_str = cpu_jiffies_str.substr(3, std::string::npos);
  std::stringstream ss(cpu_jiffies_str);
  uint32_t cpu_jiffies;
  uint32_t current_jiffy;
  while (ss >> current_jiffy)
  {
    cpu_jiffies += current_jiffy;
  }
  //    std::cout<<cpu_jiffies<<std::endl;
  return cpu_jiffies;
}

static inline int get_cpu_count()
{
  std::ifstream proc_stat("/proc/cpuinfo");
  std::string line;
  std::string proc_name{"processor"};
  int cpu_no{0};
  while (std::getline(proc_stat, line))
  {
    if (line.find(proc_name) != std::string::npos)
    {
      cpu_no = std::stoi(line.substr(line.length() - 2));
    }
  }
  cpu_no += 1;
  return cpu_no;
}

static inline int32_t get_process_memory_load_kb()
{
  std::ifstream proc_status("/proc/self/status");
  std::string mem_str;
  for (int i = 0; i < 22; i++)
  {
    std::getline(proc_status, mem_str);
  }
  std::istringstream ss(mem_str);
  std::string n;
  int32_t mem_usage{0};
  if (ss >> n >> n)
  {
    mem_usage = std::stoi(n);
  }
  return mem_usage;
}

static inline uint32_t get_proc_jiffies(bool whit_child)
{
  std::ifstream proc_stat("/proc/self/stat");
  std::string proc_jiffies_str;
  std::getline(proc_stat, proc_jiffies_str);
  std::istringstream ss(proc_jiffies_str);
  std::string temp;
  uint32_t utime{0};
  uint32_t stime{0};
  uint32_t cutime{0};
  uint32_t cstime{0};
  uint32_t index = 1;
  while (ss >> temp)
  {
    if (index == 14)
    {
      utime = std::stoi(temp);
    }
    else if (index == 15)
    {
      stime = std::stoi(temp);
    }
    if (whit_child)
    {
      if (index == 16)
      {
        cutime = std::stoi(temp);
      }
      else if (index == 17)
      {
        cstime = std::stoi(temp);
      }
    }
    index++;
  }
  uint32_t proc_time = utime + stime + cutime + cstime;
  return proc_time;
}

static inline ProcStat *get_proc_stat_info()
{
  // Allocate and initialize the struct, open the file
  FILE *procfp = std::fopen("/proc/self/stat", "r");
  ProcStat *ps = (ProcStat *)std::calloc(sizeof(ProcStat), 1);
  ps->comm = (char *)std::calloc(sizeof(char), 200);

  // Do the read, feel bad about this format string
  std::fscanf(procfp,
              "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu "
              "%ld %ld %ld %ld %ld %ld %llu %lu %ld %lu "
              "%p %p %p %p %p %lu %lu %lu %lu %p %lu %lu "
              "%d %d %u %u %llu %lu %ld %p %p %p %p %p %p %p %d",
              &ps->pid, ps->comm, &ps->state, &ps->ppid, &ps->pgrp, &ps->session,
              &ps->tty_nr, &ps->tpgid, &ps->flags, &ps->minflt, &ps->cminflt,
              &ps->majflt, &ps->cmajflt, &ps->utime, &ps->stime, &ps->cutime,
              &ps->cstime, &ps->priority, &ps->nice, &ps->num_threads,
              &ps->itrealvalue, &ps->starttime, &ps->vsize, &ps->rss, &ps->rsslim,
              &ps->startcode, &ps->endcode, &ps->startstack, &ps->kstkesp,
              &ps->kstkeip, &ps->signal, &ps->blocked, &ps->sigignore, &ps->sigcatch,
              &ps->wchan, &ps->nswap, &ps->cnswap, &ps->exit_signal, &ps->processor,
              &ps->rt_priority, &ps->policy, &ps->delayacct_blkio_ticks,
              &ps->guest_time, &ps->cguest_time, &ps->start_data, &ps->end_data,
              &ps->start_brk, &ps->arg_start, &ps->arg_end, &ps->env_start,
              &ps->env_end, &ps->exit_code);
  std::fclose(procfp);
  return ps;
}

static inline Uptime *get_uptime_info()
{
  // Allocate and initialize the struct, open the file
  FILE *uptimefp = std::fopen("/proc/uptime", "r");
  Uptime *ps = (Uptime *)std::calloc(sizeof(Uptime), 1);
  // Do the read, feel bad about this format string
  std::fscanf(uptimefp,
              "%f %f",
              &ps->uptime, &ps->idle);
  std::fclose(uptimefp);
  return ps;
}

static inline void delete_proc_stat_info(ProcStat *ps)
{
  std::free(ps->comm);
  std::free(ps);
}

static inline void delete_uptime_info(Uptime *ps)
{
  std::free(ps);
}

#endif // PROC_STAT_H