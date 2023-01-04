project('wiring', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20'])

src = ['src/main.cpp', 'src/data.h', 'src/basic.h', 'src/basic.cpp']

asa047 = ['lib/asa047/asa047.hpp', 'lib/asa047/asa047.cpp']

code = [src, asa047]

exe = executable('wiring', code)
