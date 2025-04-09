# Copyright 2024-2025 DreamWorks Animation LLC
# SPDX-License-Identifier: Apache-2.0

# -*- coding: utf-8 -*-
import os

name = 'moonshine_usd'

if 'early' not in locals() or not callable(early):
    def early(): return lambda x: x

@early()
def version():
    """
    Increment the build in the version.
    """
    _version = '14.9'
    from rezbuild import earlybind
    return earlybind.version(this, _version)

description = 'USD Shaders for Moonray'

authors = [
    'PSW Rendering and Shading',
    'moonbase-dev@dreamworks.com',
    'Ron.Woods@dreamworks.com'
]

help = ('For assistance, '
        "please contact the folio's owner at: moonbase-dev@dreamworks.com")

variants = [
    ['os-rocky-9', 'opt_level-optdebug', 'refplat-vfx2023.1', 'gcc-11.x',       'usd_core-0.22.5.x'],
    ['os-rocky-9', 'opt_level-debug',    'refplat-vfx2023.1', 'gcc-11.x',       'usd_core-0.22.5.x'],
    ['os-rocky-9', 'opt_level-optdebug', 'refplat-vfx2023.1', 'clang-17.0.6.x', 'usd_core-0.22.5.x'],
    ['os-rocky-9', 'opt_level-optdebug', 'refplat-vfx2023.1', 'gcc-11.x',       'usd_core-0.23.8.x'],
    ['os-rocky-9', 'opt_level-optdebug', 'refplat-vfx2022.0', 'gcc-9.3.x.1',    'usd_core-0.22.5.x'],
    ['os-rocky-9', 'opt_level-optdebug', 'refplat-vfx2024.0', 'gcc-11.x',       'usd_core-0.24.3.x'],

    ['os-CentOS-7', 'opt_level-optdebug', 'refplat-vfx2022.0', 'gcc-9.3.x.1', 'usd_core-0.22.5.x'],
    ['os-CentOS-7', 'opt_level-debug',    'refplat-vfx2022.0', 'gcc-9.3.x.1', 'usd_core-0.22.5.x'],
]

conf_rats_variants = variants[0:2]
conf_CI_variants = variants

requires = [
    'moonray-17.9',
    'moonshine-14.9',
    'scene_rdl2-15.6',
]

private_build_requires = [
    'cmake_modules-1.0',
    'usd_imaging',
    'cppunit',
    'ispc-1.20.0.x',
]

commandstr = lambda i: "cd build/"+os.path.join(*variants[i])+"; ctest -j $(nproc)"
testentry = lambda i: ("variant%d" % i,
                       { "command": commandstr(i),
                         "requires": ["cmake-3.23"] + variants[i] } )
testlist = [testentry(i) for i in range(len(variants))]
tests = dict(testlist)

def commands():
    prependenv('CMAKE_PREFIX_PATH', '{root}')
    prependenv('SOFTMAP_PATH', '{root}')
    prependenv('MOONRAY_DSO_PATH', '{root}/rdl2dso')
    prependenv('RDL2_DSO_PATH', '{root}/rdl2dso')
    prependenv('LD_LIBRARY_PATH', '{root}/lib')
    prependenv('PATH', '{root}/bin')
    prependenv('MOONRAY_CLASS_PATH', '{root}/coredata')

uuid = 'bb291e0f-ce27-445d-b280-320a5f23e9b9'

config_version = 0
