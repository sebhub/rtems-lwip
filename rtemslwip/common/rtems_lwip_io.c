/*
 *
 * RTEMS Project (https://www.rtems.org/)
 *
 * Copyright (c) 2021 Vijay Kumar Banerjee <vijay@rtems.org>.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#define LWIP_COMPAT_SOCKETS 0

#include <string.h>
#include <stdarg.h>
/* #include <stdlib.h> */
#include <stdio.h>
#include <errno.h>

#include <rtems.h>
#include <rtems/libio_.h>
#include <rtems/error.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/sysctl.h>

#include <rtems/thread.h>

#include <lwip/tcpip.h>
#include <lwip/api.h>
#include <lwip/netbuf.h>
#include <lwip/netdb.h>
#include <lwip/netifapi.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <sys/errno.h>

#include <lwip/init.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include <netif/etharp.h>

#include <string.h>

#include "lwip/tcpip.h"
//#include "arch/eth_lwip.h"
#include "lwip/api.h"
#include "lwip/netbuf.h"
#include "lwip/netdb.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "rtems_lwip_int.h"

static const rtems_filesystem_file_handlers_r rtems_lwip_socket_handlers;

static rtems_recursive_mutex rtems_lwip_mutex =
  RTEMS_RECURSIVE_MUTEX_INITIALIZER( "_LWIP" );

void rtems_lwip_semaphore_obtain( void )
{
    rtems_recursive_mutex_lock( &rtems_lwip_mutex );
}

/*
 * Release network mutex
 */
void rtems_lwip_semaphore_release( void )
{
    rtems_recursive_mutex_unlock( &rtems_lwip_mutex );
}

static inline int rtems_lwip_iop_to_lwipfd( rtems_libio_t *iop )
{
  if ( iop == NULL ) {
    errno = EBADF;

    return -1;
  }

/*
   if ((rtems_libio_iop_flags(iop) & LIBIO_FLAGS_OPEN) == 0) {
    errno = EBADF;
    return -1;
   }
 */

  if ( iop->pathinfo.handlers != &rtems_lwip_socket_handlers ) {
    errno = ENOTSOCK;

    return -1;
  }

  return iop->data0;
}

/*
 * Convert an RTEMS file descriptor to a LWIP socket pointer.
 */
int rtems_lwip_sysfd_to_lwipfd( int fd )
{
  if ( (uint32_t) fd >= rtems_libio_number_iops ) {
    errno = EBADF;

    return -1;
  }

  return rtems_lwip_iop_to_lwipfd( rtems_libio_iop( fd ) );
}

/*
 * Create an RTEMS file descriptor for a socket
 */
static int rtems_lwip_make_sysfd_from_lwipfd( int lfwipfd )
{
  rtems_libio_t *iop;
  int            fd;

  iop = rtems_libio_allocate();

  if ( iop == 0 )
    rtems_set_errno_and_return_minus_one( ENFILE );

  fd = rtems_libio_iop_to_descriptor( iop );
  iop->data0 = lfwipfd;
  iop->data1 = NULL;
  iop->pathinfo.handlers = &rtems_lwip_socket_handlers;
  iop->pathinfo.mt_entry = &rtems_filesystem_null_mt_entry;
  rtems_filesystem_location_add_to_mt_entry( &iop->pathinfo );
  rtems_libio_iop_flags_set( iop, LIBIO_FLAGS_READ_WRITE | LIBIO_FLAGS_OPEN );

  return fd;
}

/*
 *********************************************************************
 *                       BSD-style entry points                      *
 *********************************************************************
 */
int socket(
  int domain,
  int type,
  int protocol
)
{
  int fd;
  int lwipfd;

  if ( domain == rtems_lwip_sysdefs_PF_UNSPEC ) {
    domain = PF_UNSPEC;
  } else if ( domain == rtems_lwip_sysdefs_PF_INET ) {
    domain = PF_INET;
  } else if ( domain == rtems_lwip_sysdefs_PF_INET6 ) {
    domain = PF_INET6;
  } else {
    errno = EINVAL;

    return -1;
  }

  if ( type == rtems_lwip_sysdefs_SOCK_STREAM ) {
    type = SOCK_STREAM;
  } else if ( type == rtems_lwip_sysdefs_SOCK_DGRAM ) {
    type = SOCK_DGRAM;
  } else {
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();

  lwipfd = lwip_socket( domain, type, 0 );

  if ( lwipfd < 0 ) {
    fd = -1;
  } else {
    fd = rtems_lwip_make_sysfd_from_lwipfd( lwipfd );

    if ( fd < 0 ) {
      lwip_close( lwipfd );
    }
  }

  rtems_lwip_semaphore_release();

  return fd;
}

int bind(
  int                    s,
  const struct sockaddr *name,
  socklen_t              namelen
)
{
  int                ret = -1;
  int                lwipfd;
  int                family;
  struct sockaddr   *lwipname;
  socklen_t          lwipnamelen;
  struct sockaddr_in so_in;

 #if LWIP_IPV6
  struct sockaddr_in6 so_in6;
 #endif

  family = rtems_lwip_sysdefs_sockaddr_get_family( name );

  if ( ( family == rtems_lwip_sysdefs_AF_INET ) &&
       ( namelen >= rtems_lwip_sysdefs_sockaddr_in_size ) ) {
    so_in.sin_len = sizeof( so_in );
    so_in.sin_family = PF_INET;
    so_in.sin_port = rtems_lwip_sysdefs_sockaddr_in_get_sin_port( name );
    so_in.sin_addr.s_addr =
      rtems_lwip_sysdefs_sockaddr_in_get_sin_addr( name );
    lwipname = (struct sockaddr *) &so_in;
    lwipnamelen = so_in.sin_len;
 #if LWIP_IPV6
  } else if ( ( family == rtems_lwip_sysdefs_AF_INET6 ) &&
              ( namelen >= rtems_lwip_sysdefs_sockaddr_in6_size ) ) {
    so_in6.sin6_len = sizeof( so_in6 );
    so_in6.sin6_family = PF_INET6;
    so_in6.sin6_port = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_port( name );
    memcpy( &so_in6.sin6_addr,
      rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr_const( name ),
      sizeof( so_in6.sin6_addr ) );
    so_in6.sin6_flowinfo = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_flowinfo(
      name );
    so_in6.sin6_scope_id = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_scope_id(
      name );
    lwipname = (struct sockaddr *) &so_in6;
    lwipnamelen = so_in6.sin6_len;
 #endif
  } else {
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    ret = lwip_bind( lwipfd, lwipname, lwipnamelen );
  }

  rtems_lwip_semaphore_release();

  return ret;
}

int connect(
  int                    s,
  const struct sockaddr *name,
  socklen_t              namelen
)
{
  int                ret = -1;
  int                lwipfd;
  int                family;
  struct sockaddr   *lwipname;
  socklen_t          lwipnamelen;
  struct sockaddr_in so_in;

 #if LWIP_IPV6
  struct sockaddr_in6 so_in6;
 #endif

  family = rtems_lwip_sysdefs_sockaddr_get_family( name );

  if ( ( family == rtems_lwip_sysdefs_AF_INET ) &&
       ( namelen >= rtems_lwip_sysdefs_sockaddr_in_size ) ) {
    so_in.sin_len = sizeof( so_in );
    so_in.sin_family = AF_INET;
    so_in.sin_port = rtems_lwip_sysdefs_sockaddr_in_get_sin_port( name );
    so_in.sin_addr.s_addr =
      rtems_lwip_sysdefs_sockaddr_in_get_sin_addr( name );
    lwipname = (struct sockaddr *) &so_in;
    lwipnamelen = so_in.sin_len;
 #if LWIP_IPV6
  } else if ( ( family == rtems_lwip_sysdefs_AF_INET6 ) &&
              ( namelen >= rtems_lwip_sysdefs_sockaddr_in6_size ) ) {
    so_in6.sin6_len = sizeof( so_in6 );
    so_in6.sin6_family = AF_INET6;
    so_in6.sin6_port = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_port( name );
    memcpy( &so_in6.sin6_addr,
      rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr_const( name ),
      sizeof( so_in6.sin6_addr ) );
    so_in6.sin6_flowinfo = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_flowinfo(
      name );
    so_in6.sin6_scope_id = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_scope_id(
      name );
    lwipname = (struct sockaddr *) &so_in6;
    lwipnamelen = so_in6.sin6_len;
 #endif
  } else {
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    ret = lwip_connect( lwipfd, lwipname, lwipnamelen );
  }

  rtems_lwip_semaphore_release();

  return ret;
}

int listen(
  int s,
  int backlog
)
{
  int ret = -1;
  int lwipfd;

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    ret = lwip_listen( lwipfd, backlog );
  }

  rtems_lwip_semaphore_release();

  return ret;
}

int accept(
  int              s,
  struct sockaddr *name,
  socklen_t       *namelen
)
{
  int                ret = -1;
  int                lwipfd;
  struct sockaddr   *lwipname = name;
  socklen_t          lwipnamelen = *namelen;
  struct sockaddr_in so_in;

 #if LWIP_IPV6
  struct sockaddr_in6 so_in6;
 #endif

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {

    rtems_lwip_semaphore_release();
    lwipfd = lwip_accept( lwipfd, lwipname, &lwipnamelen );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  if ( lwipfd < 0 )
    return lwipfd;

  if ( lwipname->sa_family == AF_INET ) {
    memcpy( &so_in, lwipname, sizeof( so_in ) );
    rtems_lwip_sysdefs_sockaddr_set_len( name,
      rtems_lwip_sysdefs_sockaddr_in_size );
    rtems_lwip_sysdefs_sockaddr_set_family( name, rtems_lwip_sysdefs_AF_INET );
    rtems_lwip_sysdefs_sockaddr_in_set_sin_port( name, so_in.sin_port );
    rtems_lwip_sysdefs_sockaddr_in_set_sin_addr( name, so_in.sin_addr.s_addr );
 #if LWIP_IPV6
  } else if ( lwipname->sa_family == AF_INET6 ) {
    memcpy( &so_in6, lwipname, sizeof( so_in6 ) );
    rtems_lwip_sysdefs_sockaddr_set_len( name,
      rtems_lwip_sysdefs_sockaddr_in6_size );
    rtems_lwip_sysdefs_sockaddr_set_family( name,
      rtems_lwip_sysdefs_AF_INET6 );

    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_port( name, so_in6.sin6_port );
    memcpy( rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr( name ),
      &so_in6.sin6_addr,
      sizeof( so_in6.sin6_addr ) );
    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_flowinfo( name,
      so_in6.sin6_flowinfo );
    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_scope_id( name,
      so_in6.sin6_scope_id );
 #endif
  } else {
    lwip_close( lwipfd );
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();
  ret = rtems_lwip_make_sysfd_from_lwipfd( lwipfd );

  if ( ret < 0 ) {
    rtems_lwip_semaphore_release();
    lwip_close( lwipfd );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  *namelen = rtems_lwip_sysdefs_sockaddr_get_len( name );

  return ret;
}

/*
 *  Shutdown routine
 */

int shutdown(
  int s,
  int how
)
{
  int ret = -1;
  int lwipfd;

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    rtems_lwip_semaphore_release();
    ret = lwip_shutdown( lwipfd, how );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

ssize_t recv(
  int    s,
  void  *buf,
  size_t len,
  int    flags
)
{
  int ret = -1;
  int lwipfd;

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    rtems_lwip_semaphore_release();
    ret = lwip_recv( lwipfd, buf, len, flags );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

ssize_t send(
  int         s,
  const void *buf,
  size_t      len,
  int         flags
)
{
  int ret = -1;
  int lwipfd;

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    rtems_lwip_semaphore_release();
    ret = lwip_send( lwipfd, buf, len, flags );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

ssize_t recvfrom(
  int              s,
  void            *buf,
  size_t           len,
  int              flags,
  struct sockaddr *name,
  socklen_t       *namelen
)
{
  int                ret = -1;
  int                lwipfd;
  struct sockaddr   *lwipname = name;
  socklen_t          lwipnamelen = *namelen;
  struct sockaddr_in so_in;

 #if LWIP_IPV6
  struct sockaddr_in6 so_in6;
 #endif

  if ( name == NULL )
    return recv( s, buf, len, flags );

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    rtems_lwip_semaphore_release();
    lwipfd = lwip_recvfrom( lwipfd, buf, len, flags, lwipname, &lwipnamelen );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  if ( lwipfd < 0 )
    return lwipfd;

  if ( lwipname->sa_family == AF_INET ) {
    memcpy( &so_in, lwipname, sizeof( so_in ) );
    rtems_lwip_sysdefs_sockaddr_set_len( name,
      rtems_lwip_sysdefs_sockaddr_in_size );
    rtems_lwip_sysdefs_sockaddr_set_family( name, rtems_lwip_sysdefs_AF_INET );
    rtems_lwip_sysdefs_sockaddr_in_set_sin_port( name, so_in.sin_port );
    rtems_lwip_sysdefs_sockaddr_in_set_sin_addr( name, so_in.sin_addr.s_addr );
 #if LWIP_IPV6
  } else if ( lwipname->sa_family == AF_INET6 ) {
    memcpy( &so_in6, lwipname, sizeof( so_in6 ) );
    rtems_lwip_sysdefs_sockaddr_set_len( name,
      rtems_lwip_sysdefs_sockaddr_in6_size );
    rtems_lwip_sysdefs_sockaddr_set_family( name,
      rtems_lwip_sysdefs_AF_INET6 );

    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_port( name, so_in6.sin6_port );
    memcpy( rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr( name ),
      &so_in6.sin6_addr,
      sizeof( so_in6.sin6_addr ) );
    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_flowinfo( name,
      so_in6.sin6_flowinfo );
    rtems_lwip_sysdefs_sockaddr_in6_set_sin6_scope_id( name,
      so_in6.sin6_scope_id );
 #endif
  } else {
    lwip_close( lwipfd );
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();
  ret = rtems_lwip_make_sysfd_from_lwipfd( lwipfd );

  if ( ret < 0 ) {
    lwip_close( lwipfd );
  }

  rtems_lwip_semaphore_release();

  *namelen = rtems_lwip_sysdefs_sockaddr_get_len( name );

  return ret;
}

ssize_t sendto(
  int                    s,
  const void            *buf,
  size_t                 len,
  int                    flags,
  const struct sockaddr *name,
  socklen_t              namelen
)
{
  int                ret = -1;
  int                lwipfd;
  int                family;
  struct sockaddr   *lwipname;
  socklen_t          lwipnamelen;
  struct sockaddr_in so_in;

 #if LWIP_IPV6
  struct sockaddr_in6 so_in6;
 #endif

  if ( name == NULL )
    return send( s, buf, len, flags );

  family = rtems_lwip_sysdefs_sockaddr_get_family( name );

  if ( ( family == rtems_lwip_sysdefs_AF_INET ) &&
       ( namelen >= rtems_lwip_sysdefs_sockaddr_in_size ) ) {
    so_in.sin_len = sizeof( so_in );
    so_in.sin_family = AF_INET;
    so_in.sin_port = rtems_lwip_sysdefs_sockaddr_in_get_sin_port( name );
    so_in.sin_addr.s_addr =
      rtems_lwip_sysdefs_sockaddr_in_get_sin_addr( name );
    lwipname = (struct sockaddr *) &so_in;
    lwipnamelen = so_in.sin_len;
 #if LWIP_IPV6
  } else if ( ( family == rtems_lwip_sysdefs_AF_INET6 ) &&
              ( namelen >= rtems_lwip_sysdefs_sockaddr_in6_size ) ) {
    so_in6.sin6_len = sizeof( so_in6 );
    so_in6.sin6_family = AF_INET6;
    so_in6.sin6_port = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_port( name );
    memcpy( &so_in6.sin6_addr,
      rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr_const( name ),
      sizeof( so_in6.sin6_addr ) );
    so_in6.sin6_flowinfo = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_flowinfo(
      name );
    so_in6.sin6_scope_id = rtems_lwip_sysdefs_sockaddr_in6_get_sin6_scope_id(
      name );
    lwipname = (struct sockaddr *) &so_in6;
    lwipnamelen = so_in6.sin6_len;
 #endif
  } else {
    errno = EINVAL;

    return -1;
  }

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) >= 0 ) {
    rtems_lwip_semaphore_release();
    ret = lwip_sendto( lwipfd, buf, len, flags, lwipname, lwipnamelen );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

#if 0

/*
 * All `transmit' operations end up calling this routine.
 */
ssize_t sendmsg(
  int                  s,
  const struct msghdr *mp,
  int                  flags
)
{
  int           ret = -1;
  int           error;
  struct uio    auio;
  struct iovec *iov;
  int           lwipfd;
  struct mbuf  *to;
  struct mbuf  *control = NULL;
  int           i;
  int           len;

  rtems_lwip_semaphore_obtain();

  if ( ( so = rtems_lwip_sysfd_to_lwipfd( s ) ) == NULL ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  auio.uio_iov = mp->msg_iov;
  auio.uio_iovcnt = mp->msg_iovlen;
  auio.uio_segflg = UIO_USERSPACE;
  auio.uio_rw = UIO_WRITE;
  auio.uio_offset = 0;
  auio.uio_resid = 0;
  iov = mp->msg_iov;

  for ( i = 0; i < mp->msg_iovlen; i++, iov++ ) {
    if ( ( auio.uio_resid += iov->iov_len ) < 0 ) {
      errno = EINVAL;
      rtems_lwip_semaphore_release();

      return -1;
    }
  }

  if ( mp->msg_name ) {
    error = sockargstombuf( &to, mp->msg_name, mp->msg_namelen, MT_SONAME );

    if ( error ) {
      errno = error;
      rtems_lwip_semaphore_release();

      return -1;
    }
  } else {
    to = NULL;
  }

  if ( mp->msg_control ) {
    if ( mp->msg_controllen < sizeof( struct cmsghdr ) ) {
      errno = EINVAL;

      if ( to )
        m_freem( to );

      rtems_lwip_semaphore_release();

      return -1;
    }

    sockargstombuf( &control, mp->msg_control, mp->msg_controllen,
      MT_CONTROL );
  } else {
    control = NULL;
  }

  len = auio.uio_resid;
  error = sosend( so, to, &auio, (struct mbuf *) 0, control, flags );

  if ( error ) {
    if ( auio.uio_resid != len && ( error == EINTR || error == EWOULDBLOCK ) )
      error = 0;
  }

  if ( error )
    errno = error;
  else
    ret = len - auio.uio_resid;

  if ( to )
    m_freem( to );

  rtems_lwip_semaphore_release();

  return ( ret );
}

/*
 * All `receive' operations end up calling this routine.
 */
ssize_t recvmsg(
  int            s,
  struct msghdr *mp,
  int            flags
)
{
  int           ret = -1;
  int           error;
  struct uio    auio;
  struct iovec *iov;
  int           lwipfd;
  struct mbuf  *from = NULL, *control = NULL;
  int           i;
  int           len;

  rtems_lwip_semaphore_obtain();

  if ( ( so = rtems_lwip_sysfd_to_lwipfd( s ) ) == NULL ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  auio.uio_iov = mp->msg_iov;
  auio.uio_iovcnt = mp->msg_iovlen;
  auio.uio_segflg = UIO_USERSPACE;
  auio.uio_rw = UIO_READ;
  auio.uio_offset = 0;
  auio.uio_resid = 0;
  iov = mp->msg_iov;

  for ( i = 0; i < mp->msg_iovlen; i++, iov++ ) {
    if ( ( auio.uio_resid += iov->iov_len ) < 0 ) {
      errno = EINVAL;
      rtems_lwip_semaphore_release();

      return -1;
    }
  }

  len = auio.uio_resid;
  mp->msg_flags = flags;
  error = soreceive( so, &from, &auio, (struct mbuf **) NULL,
    mp->msg_control ? &control : (struct mbuf **) NULL,
    &mp->msg_flags );

  if ( error ) {
    if ( auio.uio_resid != len && ( error == EINTR || error == EWOULDBLOCK ) )
      error = 0;
  }

  if ( error ) {
    errno = error;
  } else {
    ret = len - auio.uio_resid;

    if ( mp->msg_name ) {
      len = mp->msg_namelen;

      if ( ( len <= 0 ) || ( from == NULL ) ) {
        len = 0;
      } else {
        if ( len > from->m_len )
          len = from->m_len;

        memcpy( mp->msg_name, mtod( from, caddr_t ), len );
      }

      mp->msg_namelen = len;
    }

    if ( mp->msg_control ) {
      struct mbuf *m;
      void        *ctlbuf;

      len = mp->msg_controllen;
      m = control;
      mp->msg_controllen = 0;
      ctlbuf = mp->msg_control;

      while ( m && ( len > 0 ) ) {
        unsigned int tocopy;

        if ( len >= m->m_len )
          tocopy = m->m_len;
        else {
          mp->msg_flags |= MSG_CTRUNC;
          tocopy = len;
        }

        memcpy( ctlbuf, mtod( m, caddr_t ), tocopy );
        ctlbuf += tocopy;
        len -= tocopy;
        m = m->m_next;
      }

      mp->msg_controllen = ctlbuf - mp->msg_control;
    }
  }

  if ( from )
    m_freem( from );

  if ( control )
    m_freem( control );

  rtems_lwip_semaphore_release();

  return ( ret );
}

#endif

int setsockopt(
  int         s,
  int         level,
  int         name,
  const void *val,
  socklen_t   len
)
{

#if 0
  int          lwipfd;
  struct mbuf *m = NULL;
  int          error;

  rtems_lwip_semaphore_obtain();

  if ( ( so = rtems_lwip_sysfd_to_lwipfd( s ) ) == NULL ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  if ( len > MLEN ) {
    errno = EINVAL;
    rtems_lwip_semaphore_release();

    return -1;
  }

  if ( val ) {
    error = sockargstombuf( &m, val, len, MT_SOOPTS );

    if ( error ) {
      errno = error;
      rtems_lwip_semaphore_release();

      return -1;
    }
  }

  error = sosetopt( so, level, name, m );

  if ( error ) {
    errno = error;
    rtems_lwip_semaphore_release();

    return -1;
  }

  rtems_lwip_semaphore_release();
#endif
  return 0;
}

#if 0

int getsockopt(
  int        s,
  int        level,
  int        name,
  void      *aval,
  socklen_t *avalsize
)
{
  int          lwipfd;
  struct mbuf *m = NULL, *m0;
  char        *val = aval;
  int          i, op, valsize;
  int          error;

  rtems_lwip_semaphore_obtain();

  if ( ( so = rtems_lwip_sysfd_to_lwipfd( s ) ) == NULL ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  if ( val )
    valsize = *avalsize;
  else
    valsize = 0;

  if ( ( ( error =
             sogetopt( so, level, name,
               &m ) ) == 0 ) && val && valsize && m ) {
    op = 0;

    while ( m && op < valsize ) {
      i = valsize - op;

      if ( i > m->m_len )
        i = m->m_len;

      memcpy( val, mtod( m, caddr_t ), i );
      op += i;
      val += i;
      m0 = m;
      MFREE( m0, m );
    }

    *avalsize = op;
  }

  if ( m != NULL )
    (void) m_free( m );

  if ( error ) {
    errno = error;
    rtems_lwip_semaphore_release();

    return -1;
  }

  rtems_lwip_semaphore_release();

  return 0;
}

static int getpeersockname(
  int              s,
  struct sockaddr *name,
  socklen_t       *namelen,
  int              pflag
)
{
  int          lwipfd;
  struct mbuf *m;
  int          len = *namelen;
  int          error;

  rtems_lwip_semaphore_obtain();

  if ( ( so = rtems_lwip_sysfd_to_lwipfd( s ) ) == NULL ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  m = m_getclr( M_WAIT, MT_SONAME );

  if ( m == NULL ) {
    errno = ENOBUFS;
    rtems_lwip_semaphore_release();

    return -1;
  }

  if ( pflag )
    error = ( *so->so_proto->pr_usrreqs->pru_peeraddr )( so, m );
  else
    error = ( *so->so_proto->pr_usrreqs->pru_sockaddr )( so, m );

  if ( error ) {
    m_freem( m );
    errno = error;
    rtems_lwip_semaphore_release();

    return -1;
  }

  if ( len > m->m_len ) {
    len = m->m_len;
    *namelen = len;
  }

  memcpy( name, mtod( m, caddr_t ), len );
  m_freem( m );
  rtems_lwip_semaphore_release();

  return 0;
}

#endif

int getpeername(
  int              s,
  struct sockaddr *name,
  socklen_t       *namelen
)
{
  int          lwipfd;
  int          error;

  rtems_lwip_semaphore_obtain();

  if ( ( lwipfd = rtems_lwip_sysfd_to_lwipfd( s ) ) < 0 ) {
    rtems_lwip_semaphore_release();

    return -1;
  }

  error = lwip_getpeername(lwipfd, name, namelen);

  rtems_lwip_semaphore_release();

  return error;
}

#if 0

int getsockname(
  int              s,
  struct sockaddr *name,
  socklen_t       *namelen
)
{
  return getpeersockname( s, name, namelen, 0 );
}

int sysctl(
  const int  *name,
  u_int       namelen,
  void       *oldp,
  size_t     *oldlenp,
  const void *newp,
  size_t      newlen
)
{
  int    error;
  size_t j;

  rtems_lwip_semaphore_obtain();
  error =
    userland_sysctl( 0, name, namelen, oldp, oldlenp, 1, newp, newlen, &j );
  rtems_lwip_semaphore_release();

  if ( oldlenp )
    *oldlenp = j;

  if ( error ) {
    errno = error;

    return -1;
  }

  return 0;
}

#endif

/*
 ************************************************************************
 *                      RTEMS I/O HANDLER ROUTINES                      *
 ************************************************************************
 */
static int rtems_lwip_close( rtems_libio_t *iop )
{
  int lwipfd;
  int ret;

  rtems_lwip_semaphore_obtain();
  lwipfd = rtems_lwip_iop_to_lwipfd( iop );

  if ( lwipfd < 0 ) {
    ret = -1;
  } else {
    rtems_lwip_semaphore_release();
    ret = lwip_close( lwipfd );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

static ssize_t rtems_lwip_read(
  rtems_libio_t *iop,
  void          *buffer,
  size_t         count
)
{
  int lwipfd;
  int ret;

  rtems_lwip_semaphore_obtain();
  lwipfd = rtems_lwip_iop_to_lwipfd( iop );

  if ( lwipfd < 0 ) {
    ret = -1;
  } else {
    rtems_lwip_semaphore_release();
    ret = lwip_read( lwipfd, buffer, count );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

static ssize_t rtems_lwip_write(
  rtems_libio_t *iop,
  const void    *buffer,
  size_t         count
)
{
  int lwipfd;
  int ret;

  rtems_lwip_semaphore_obtain();
  lwipfd = rtems_lwip_iop_to_lwipfd( iop );

  if ( lwipfd < 0 ) {
    ret = -1;
  } else {
    rtems_lwip_semaphore_release();
    ret = lwip_write( lwipfd, buffer, count );
    rtems_lwip_semaphore_obtain();
  }

  rtems_lwip_semaphore_release();

  return ret;
}

int so_ioctl(
  rtems_libio_t *iop,
  int            lwipfd,
  uint32_t       command,
  void          *buffer
)
{
#if 0

  switch ( command ) {
    case FIONBIO:

      if ( *(int *) buffer ) {
        iop->flags |= O_NONBLOCK;
        so->so_state |= SS_NBIO;
      } else {
        iop->flags &= ~O_NONBLOCK;
        so->so_state &= ~SS_NBIO;
      }

      return 0;

    case FIONREAD:
      *(int *) buffer = so->so_rcv.sb_cc;

      return 0;
  }

  if ( IOCGROUP( command ) == 'i' )
    return ifioctl( so, command, buffer, NULL );

  if ( IOCGROUP( command ) == 'r' )
    return rtioctl( command, buffer, NULL );

  return ( *so->so_proto->pr_usrreqs->pru_control )( so, command, buffer, 0 );
#endif

  return -1;
}

static int rtems_lwip_ioctl(
  rtems_libio_t  *iop,
  ioctl_command_t command,
  void           *buffer
)
{
#if 0
  int lwipfd;
  int error;

  rtems_lwip_semaphore_obtain();

  if ( ( so = iop->data1 ) == NULL ) {
    errno = EBADF;
    rtems_lwip_semaphore_release();

    return -1;
  }

  error = so_ioctl( iop, so, command, buffer );
  rtems_lwip_semaphore_release();

  if ( error ) {
    errno = error;

    return -1;
  }

  return 0;
#endif

  return -1;
}

static int rtems_lwip_fcntl(
  rtems_libio_t *iop,
  int            cmd
)
{
#if 0
  int lwipfd;

  if ( cmd == F_SETFL ) {
    rtems_lwip_semaphore_obtain();

    if ( ( so = iop->data1 ) == NULL ) {
      rtems_lwip_semaphore_release();

      return EBADF;
    }

    if ( rtems_libio_iop_is_no_delay( iop ) )
      so->so_state |= SS_NBIO;
    else
      so->so_state &= ~SS_NBIO;

    rtems_lwip_semaphore_release();
  }

  return 0;
#endif

  return -1;
}

static int rtems_lwip_fstat(
  const rtems_filesystem_location_info_t *loc,
  struct stat                            *sp
)
{
  sp->st_mode = S_IFSOCK;

  return 0;
}

static const rtems_filesystem_file_handlers_r rtems_lwip_socket_handlers = {
  .open_h = rtems_filesystem_default_open,
  .close_h = rtems_lwip_close,
  .read_h = rtems_lwip_read,
  .write_h = rtems_lwip_write,
  .ioctl_h = rtems_lwip_ioctl,
  .lseek_h = rtems_filesystem_default_lseek,
  .fstat_h = rtems_lwip_fstat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_lwip_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

const char *
inet_ntop(int af, const void *src, char *dst, socklen_t size){
	    return lwip_inet_ntop(af, src, dst, size);
}
