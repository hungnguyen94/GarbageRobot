import ConfigParser

config = ConfigParser.RawConfigParser()

# When adding sections or items, add them in the reverse order of
# how you want them to be displayed in the actual file.
# In addition, please note that using RawConfigParser's and the raw
# mode of ConfigParser's respective set functions, you can assign
# non-string values to keys internally, but will receive an error
# when attempting to write to a file or when you get it in non-raw
# mode. SafeConfigParser does not allow such assignments to take place.
config.add_section('GPIO_in')
config.set('GPIO_in', 'mouth_sensor', '15')
config.set('GPIO_in', 'fill_level_pmd', '15')
config.set('GPIO_in', 'fill_level_coffee', '15')
config.set('GPIO_in', 'fill_level_residual', '15')
config.set('GPIO_in', 'stop_button', '15')

config.add_section('GPIO_out')
config.set('GPIO_out', 'class_can', '15')
config.set('GPIO_out', 'class_bottle', '15')
config.set('GPIO_out', 'class_coffee', '15')
config.set('GPIO_out', 'class_residual', '15')
config.set('GPIO_out', 'bin_full', '15')

# Writing our configuration file to 'example.cfg'
with open('cfg/rpi_pins.cfg', 'wb') as configfile:
    config.write(configfile)
