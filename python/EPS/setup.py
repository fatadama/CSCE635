from distutils.core import setup, Extension

module1 = Extension('esp',
                    sources = ['espmodule.cpp'])

setup (name = 'esp',
       version = '1.0',
       description = 'Emily Serial Protocol python extension',
       ext_modules = [module1])
       #ext_modules=[Extension('eps_pack_gps', ['epsmodule.cpp'])])
