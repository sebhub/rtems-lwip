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
import json
import os


def removeprefix(data, prefix):
    if data.startswith(prefix):
        return data[len(prefix):]
    return data


xilinx_lwip_prefix = 'embeddedsw/ThirdParty/sw_services/lwip211/src/contrib/'
xilinx_standalone_prefix = 'embeddedsw/lib/bsp/standalone/src/'

xilinx_drv_incl = [
    xilinx_lwip_prefix + 'ports/xilinx/include',
    xilinx_standalone_prefix + 'common',
    'embeddedsw/XilinxProcessorIPLib/drivers/common/src/',
    'embeddedsw/XilinxProcessorIPLib/drivers/scugic/src',
    'embeddedsw/XilinxProcessorIPLib/drivers/emacps/src',
    'rtemslwip/xilinx'
]

xilinx_aarch64_drv_incl = [
    'rtemslwip/zynqmp',
    xilinx_standalone_prefix + 'arm/ARMv8/64bit',
    xilinx_standalone_prefix + 'arm/common/gcc',
    xilinx_standalone_prefix + 'arm/common'
]

# These sources are explicitly listed instead of using walk_sources below
# because multiple BSPs of varying architecture are expected to use code from
# the embeddedsw repository.
xilinx_aarch64_driver_source = [
    xilinx_lwip_prefix + 'ports/xilinx/netif/xadapter.c',
    xilinx_lwip_prefix + 'ports/xilinx/netif/xpqueue.c',
    xilinx_lwip_prefix + 'ports/xilinx/netif/xemacpsif.c',
    xilinx_lwip_prefix + 'ports/xilinx/netif/xemacpsif_dma.c',
    xilinx_lwip_prefix + 'ports/xilinx/netif/xemacpsif_hw.c',
    xilinx_lwip_prefix + 'ports/xilinx/netif/xemacpsif_physpeed.c',
    'embeddedsw/XilinxProcessorIPLib/drivers/emacps/src/xemacps_bdring.c',
    'embeddedsw/XilinxProcessorIPLib/drivers/emacps/src/xemacps.c',
    'embeddedsw/XilinxProcessorIPLib/drivers/emacps/src/xemacps_control.c',
    'embeddedsw/XilinxProcessorIPLib/drivers/emacps/src/xemacps_intr.c',
    xilinx_standalone_prefix + 'common/xil_assert.c'
]

common_includes = [
    'lwip/src/include',
    'uLan/ports/os/rtems',
    'rtemslwip/include'
]

bsd_compat_incl = [
    'rtemslwip/bsd_compat_include'
]

common_source_files = [
    'uLan/ports/os/rtems/arch/sys_arch.c',
    'rtemslwip/common/syslog.c',
    'rtemslwip/common/rtems_lwip_io.c',
    'rtemslwip/common/network_compat.c',
    'rtemslwip/bsd_compat/netdb.c',
    'rtemslwip/bsd_compat/ifaddrs.c',
    'rtemslwip/bsd_compat/rtems-kernel-program.c'
]


def build(bld):
    source_files = []
    driver_source = []
    drv_incl = []
    arch_lib_path = rtems.arch_bsp_lib_path(bld.env.RTEMS_VERSION,
                                            bld.env.RTEMS_ARCH_BSP)
    arch = rtems.arch(bld.env.RTEMS_ARCH_BSP)
    bsp = rtems.bsp(bld.env.RTEMS_ARCH_BSP)
    with open('file-import.json', 'r') as cf:
        files = json.load(cf)
        for f in files['files-to-import']:
            if f[-2:] == '.c':
                source_files.append(os.path.join('lwip', f))

    source_files.extend(common_source_files)

    def walk_sources(path):
        return bld.path.ant_glob([path + '/**/*.c', path + '/**/*.S'])

    if arch == 'arm':
        # These files will not compile for BSPs other than TMS570
        if bsp in ['tms570ls3137_hdk', 'tms570ls3137_hdk_intram',
                   'tms570ls3137_hdk_sdram', 'tms570ls3137_hdk_with_loader']:
            drv_incl.append('uLan/ports/driver/tms570_emac')
            drv_incl.append('uLan/ports/os')
            driver_source.extend(walk_sources('uLan/ports/driver/tms570_emac'))

        # These files will only compile for BeagleBone BSPs
        if bsp in ['beagleboneblack', 'beaglebonewhite']:
            driver_source.extend(walk_sources('rtemslwip/beaglebone'))
            drv_incl.append('rtemslwip/beaglebone')
            drv_incl.append('cpsw/src/include')
            driver_source.extend(walk_sources('cpsw/src'))


    # These files will only compile for BSPs on Xilinx hardware
    is_xilinx_bsp = False
    is_aarch64_bsp = False
    is_qemu = False
    if arch == 'aarch64' and bsp in ['xilinx_zynqmp_lp64_qemu',
                                     'xilinx_zynqmp_lp64_zu3eg',
                                     'xilinx_zynqmp_ilp32_qemu',
                                     'xilinx_zynqmp_ilp32_zu3eg']:
        is_xilinx_bsp = True
        is_aarch64_bsp = True
    if bsp in ['xilinx_zynqmp_lp64_qemu', 'xilinx_zynqmp_ilp32_qemu']:
        is_qemu = True
    if is_xilinx_bsp:
        drv_incl.extend(xilinx_drv_incl)
        if is_aarch64_bsp:
            driver_source.extend(walk_sources('rtemslwip/zynqmp'))
            if is_qemu:
                driver_source.extend(walk_sources('rtemslwip/zynqmp_qemu'))
            else:
                driver_source.extend(walk_sources('rtemslwip/zynqmp_hardware'))
            driver_source.extend(xilinx_aarch64_driver_source)
            drv_incl.extend(xilinx_aarch64_drv_incl)

    lwip_obj_incl = []
    lwip_obj_incl.extend(drv_incl)
    lwip_obj_incl.extend(bsd_compat_incl)
    lwip_obj_incl.extend(common_includes)

    bld(features='c',
        target='lwip_obj',
        cflags='-g -Wall -O0',
        includes=' '.join(lwip_obj_incl),
        source=source_files,
        )

    drv_obj_incl = []
    drv_obj_incl.extend(drv_incl)
    drv_obj_incl.extend(common_includes)

    bld(features='c',
        target='driver_obj',
        cflags='-g -Wall -O0',
        includes=' '.join(drv_obj_incl),
        source=driver_source,
        )
    bld(features='c cstlib',
        target='lwip',
        cflags='-g -Wall -O0',
        use=['lwip_obj', 'driver_obj'])
    bld.install_files("${PREFIX}/" + arch_lib_path, ["liblwip.a"])

    def install_headers(root_path):
        for root, dirs, files in os.walk(root_path):
            for name in files:
                ext = os.path.splitext(name)[1]
                src_root = os.path.split(root)
                path = os.path.join(src_root[0], src_root[1])
                if ext == '.h':
                    subpath = removeprefix(removeprefix(path, root_path), "/")
                    bld.install_files(
                        "${PREFIX}/" + arch_lib_path + "/include/" + subpath,
                        os.path.join(path, name)
                    )

    [install_headers(path) for path in common_includes]
    [install_headers(path) for path in drv_incl]
    [install_headers(path) for path in bsd_compat_incl]

    test_app_incl = []
    test_app_incl.extend(drv_incl)
    test_app_incl.extend(common_includes)
    test_app_incl.append('rtemslwip/test/')
    bld.program(features='c',
                target='networking01.exe',
                source='rtemslwip/test/networking01/sample_app.c',
                cflags='-g -Wall -O0',
                use='lwip',
                lib=['rtemscpu', 'rtemsbsp', 'rtemstest', 'lwip'],
                includes=' '.join(test_app_incl))

    lib_path = os.path.join(bld.env.PREFIX, arch_lib_path)
    bld.read_stlib('telnetd', paths=[lib_path])
    bld.read_stlib('rtemstest', paths=[lib_path])
    bld.read_stlib('ftpd', paths=[lib_path])

    bld.program(features='c',
                target='telnetd01.exe',
                source='rtemslwip/test/telnetd01/init.c',
                use='telnetd lwip rtemstest ftpd',
                cflags='-g -Wall -O0',
                includes=' '.join(test_app_incl))


def add_flags(flags, new_flags):
    for flag in new_flags:
        if flag not in flags:
            flags.append(flag)


def bsp_configure(conf, arch_bsp):
    conf.env.LIB += ['m']
    section_flags = ["-fdata-sections", "-ffunction-sections"]
    add_flags(conf.env.CFLAGS, section_flags)
    add_flags(conf.env.CXXFLAGS, section_flags)
