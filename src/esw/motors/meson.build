project('src/motors', 'cpp', default_options : ['cpp_std=c++1z'])

ROS = dependency('lcm')
rapidjson = dependency('RapidJSON')

all_deps = [lcm, rapidjson]

unit_test = get_option('unit_test')

if unit_test
    install_headers('Controller.h', 'ControllerMap.h', 'I2C.h')
    src = ['test.cpp', 'ControllerMap.cpp', 'I2C.cpp', 'Controller.cpp']


    executable('src/motors',
            sources: src,
            dependencies : all_deps,
            install : true)
else
    install_headers('Controller.h', 'ControllerMap.h', 'I2C.h', 'ROSHandler.h')
    src = ['main.cpp', 'ControllerMap.cpp', 'I2C.cpp', 'ROSHandler.cpp', 'Controller.cpp']

    executable('src/motors',
            sources: src,
            dependencies : all_deps,
            install : true)

endif