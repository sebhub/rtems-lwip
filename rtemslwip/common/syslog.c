/*
 * RTEMS version of syslog and associated routines
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/thread.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>

#include <unistd.h>

static int LogStatus = LOG_CONS;
static const char *LogTag = "syslog";
static int LogFacility = LOG_USER;
static int LogMask = 0xff;

void
syslog (int pri, const char *fmt, ...)
{
	va_list ap;

	va_start (ap, fmt);
	vsyslog (pri, fmt, ap);
	va_end (ap);
}

/*
 * FIXME: Should cbuf be static?  Then we wouldn't
 *        have to worry about blowing stacks with a local variable
 *        that large.  Could make cbuf bigger, too.
 *
 * FIXME: This does not properly handle the %m format specifier which should
 *        insert the string result of strerror(errno).
 */
void
vsyslog (int pri, const char *fmt, va_list ap)
{
	int cnt;
	char cbuf[200];

	if (pri & ~(LOG_PRIMASK|LOG_FACMASK)) {
		syslog (LOG_ERR|LOG_CONS|LOG_PERROR|LOG_PID,
								"syslog: unknown facility/priority: %#x", pri);
		pri &= LOG_PRIMASK|LOG_FACMASK;
	}

	if (!(LOG_MASK(LOG_PRI(pri)) & LogMask))
		return;

	if ((pri & LOG_FACMASK) == 0)
		pri |= LogFacility;

	cnt = snprintf (cbuf, sizeof (cbuf), "<%d>", pri);
	if (LogTag && cnt < sizeof (cbuf) - 1)
		cnt += snprintf (cbuf + cnt, sizeof (cbuf) - cnt, "%s", LogTag);
	if (LogStatus & LOG_PID && cnt < sizeof (cbuf) - 1) {
		rtems_id tid;
		rtems_task_ident (RTEMS_SELF, 0, &tid);
		cnt += snprintf (cbuf + cnt, sizeof (cbuf) - cnt, "[%#lx]", (unsigned long)tid);
	}
	if (LogTag && cnt < sizeof (cbuf) - 1)
		cnt += snprintf (cbuf + cnt, sizeof (cbuf) - cnt, ": ");
	cnt += vsnprintf (cbuf + cnt, sizeof (cbuf) - cnt, fmt, ap);
	if (cnt > sizeof (cbuf) - 1)
		cnt = sizeof (cbuf) - 1;
	while (cnt > 0 && cbuf[cnt-1] == '\n')
		cbuf[--cnt] = '\0';

	if (LogStatus & LOG_PERROR)
		printf ("%s\n", cbuf);
}

int
setlogmask (int pmask)
{
	int omask;

	omask = LogMask;
	if (pmask != 0)
		LogMask = pmask;
	return (omask);
}

void
openlog(const char *ident, int option, int facility)
{
	/* Do nothing */
}
