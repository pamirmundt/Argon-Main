from distutils.core import setup, Extension

module1 = Extension('i2cmodule',
                    sources = ['i2cmodule.c'])

setup (name = 'PackageName',
       version = '1.0',
       description = 'This is a mecanum base control module over RPi2/SMBus I2C',
       ext_modules = [module1])
