#
# RTEMS Project (https://www.rtems.org/)
#
# Copyright (c) 2021 Vijay Kumar Banerjee <vijay@rtems.org>.
# All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from rtems_waf import rtems
import yaml
import os


def build(bld):
    source_files = []
    common_includes = './lwip/src/include ./uLan/ports/os/rtems ./rtemslwip/include '
    driver_source = []
    drv_incl = ' '

    arch_lib_path = rtems.arch_bsp_lib_path(bld.env.RTEMS_VERSION,
                                            bld.env.RTEMS_ARCH_BSP)
    with open('file-import.yaml', 'r') as cf:
        files = yaml.full_load(cf.read())
        for f in files['files-to-import']:
            if f[-2:] == '.c':
                source_files.append(os.path.join('./lwip', f))

    source_files.append('./uLan/ports/os/rtems/arch/sys_arch.c')
    source_files.append('./rtemslwip/common/syslog.c')
    source_files.append('./rtemslwip/common/rtems_lwip_io.c')
    source_files.append('./rtemslwip/common/rtems_lwip_sysdefs.c')
    
    #source_files.append('./lwip/ports/port/sys_arch.c')
    #source_files.append('./lwip/ports/port/perf.c')
    #source_files.append('./lwip/ports/port/netif/fifo.c')
    #source_files.append('./lwip/ports/port/netif/list.c')
    #source_files.append('./lwip/ports/port/netif/pcapif.c')
    #source_files.append('./lwip/ports/port/netif/sio.c')
    #source_files.append('./lwip/ports/port/netif/tapif.c')
    
    def walk_sources(path):
        sources = []
        for root, dirs, files in os.walk(path):
            for name in files:
                ext = os.path.splitext(name)[1]
                src_root = os.path.split(root)
                path = os.path.join(src_root[0], src_root[1])
                if ext == '.c' or ext == '.S':
                    sources.append(os.path.join(path, name))
        return sources

    # These files will not compile for BSPs other than TMS570
    if bld.env.RTEMS_ARCH_BSP.startswith('arm-rtems6-tms570ls3137_hdk'):
        drv_incl += './uLan/ports/driver/tms570_emac ./uLan/ports/os '
        driver_source.extend(walk_sources('./uLan/ports/driver/tms570_emac'))

    # These files will only compile for BeagleBone BSPs
    if bld.env.RTEMS_ARCH_BSP.startswith('arm-rtems6-beaglebone'):
        driver_source.extend(walk_sources('./rtemslwip/beaglebone'))
        drv_incl += './rtemslwip/beaglebone ./cpsw/src/include '
        driver_source.extend(walk_sources('./cpsw/src'))

    bld(features ='c',
        target='lwip_obj',
        cflags='-g -Wall -O0',
        includes=drv_incl + common_includes,
        source=source_files,
        )

    bld(features ='c',
        target='driver_obj',
        cflags='-g -Wall -O0',
        includes=drv_incl + common_includes + os.path.relpath(os.path.join(bld.env.PREFIX, arch_lib_path,'include')) ,
        source=driver_source,
        )
    bld(features='c cstlib',
        target = 'lwip',
        cflags='-g -Wall -O0',
        use=['lwip_obj', 'driver_obj'])

    bld.program(features='c',
                target='networking01.exe',
                source='./rtemslwip/test/networking01/sample_app.c',
                cflags='-g -Wall -O0',
                use='lwip',
                lib=['rtemscpu', 'rtemsbsp', 'rtemstest', 'lwip'],
                includes=drv_incl + common_includes + './rtemslwip/test/ ' + os.path.relpath(os.path.join(arch_lib_path,'include')))

    arch_lib_path = rtems.arch_bsp_lib_path(bld.env.RTEMS_VERSION,
                                            bld.env.RTEMS_ARCH_BSP)
    lib_path = os.path.join(bld.env.PREFIX, arch_lib_path)
    bld.read_stlib('telnetd', paths=[lib_path])
    bld.read_stlib('rtemstest', paths=[lib_path])
    bld.read_stlib('ftpd', paths=[lib_path])

    bld.program(features='c',
                target='telnetd01.exe',
                source='./rtemslwip/test/telnetd01/init.c',
                use='telnetd lwip rtemstest ftpd',
                cflags='-g -Wall -O0',
                includes=drv_incl + common_includes + './rtemslwip/test/ ' + os.path.relpath(os.path.join(arch_lib_path,'include')))
