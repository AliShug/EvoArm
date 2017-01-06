import os
import errno
import sys
import struct

import yaml
from jinja2 import Environment, PackageLoader

env = Environment(loader=PackageLoader('protocolgen', ''),
    trim_blocks=True,
    lstrip_blocks=True)
env.globals['ord'] = ord
code_template = env.get_template('protocol.template.cpp')
py_template = env.get_template('Protocol.template.py')

input_stream = file('PyComms\protocol.yaml')
data = yaml.load(input_stream)
data['classname'] = 'Protocol'

id_file = open('idmapping.txt', 'w')

# short identifiers must be unique
int_identifier = 0
for x in data["commands"]:
    x["short"] = struct.pack('b', int_identifier)
    id_file.write('{0} : 0x{1:02X}\n'.format(x['name'], int_identifier))
    int_identifier += 1

id_file.close()

# Render templates
code = code_template.render(**data)
# header = header_template.render(**data)
py_code = py_template.render(**data)

file('Comms/{0}.cpp'.format('Protocol'), 'w').write(code)
# file('{c}/{c}.h'.format(c=cname), 'w').write(header)
file('PyComms/{0}.py'.format('Protocol'), 'w').write(py_code)
file('../PyIK/{0}.py'.format('Protocol'), 'w').write(py_code)

# profilePath = os.environ.get('USERPROFILE')
# if profilePath != None:
#     libPath = profilePath + '/Documents/Arduino/libraries'
#     try:
#         os.makedirs(libPath+'/'+cname)
#     except OSError as exception:
#         if exception.errno != errno.EEXIST:
#             raise
#     file('{lib}/{c}/{c}.cpp'.format(lib=libPath, c=cname), 'w').write(code)
#     file('{lib}/{c}/{c}.h'.format(lib=libPath, c=cname), 'w').write(header)
# else:
#     print 'Unable to locate arduino library directory'
