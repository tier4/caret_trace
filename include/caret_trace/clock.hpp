

static __inline__
uint64_t trace_clock_read64_monotonic(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return ((uint64_t) ts.tv_sec * 1000000000ULL) + ts.tv_nsec;
	}
  return 0;
}