#include <stdio.h>
#include <syslog.h>

/* Not in the installed fastrpc headers. */
extern int adsp_default_listener_start(int argc, char **argv);

int main(void)
{
	char *args[] = {"adsprpcd", "sensorspd", "adsp", NULL};

	/* Tag the library's syslog messages. */
	openlog("caihong-sensors", LOG_PID, LOG_USER);

	/* Attach once and serve until stopped. Any return is a failure: the
	 * listener can return 0 after an allocation failure. */
	int result = adsp_default_listener_start(3, args);
	fprintf(stderr, "sensor listener returned 0x%x\n", result);
	return 1;
}
