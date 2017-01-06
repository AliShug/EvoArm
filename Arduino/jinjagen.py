import os, errno, sys, yaml
from jinja2 import Environment, PackageLoader

env = Environment(loader=PackageLoader('jinjagen', 'config'),
    trim_blocks=True,
    lstrip_blocks=True)
code_template = env.get_template('template.cpp')
header_template = env.get_template('template.h')

input_stream = file(sys.argv[1])
data = yaml.load(input_stream)
code = code_template.render(
    scriptname = __file__,
    data = data
)
header = header_template.render(
    scriptname = __file__,
    data = data
)

# Output files to class directory
cname = data['classname']
try:
    os.makedirs(cname)
except OSError as exception:
    if exception.errno != errno.EEXIST:
        raise

file('{0}/{0}.cpp'.format(cname), 'w').write(code)
file('{0}/{0}.h'.format(cname), 'w').write(header)

profilePath = os.environ.get('USERPROFILE')
if profilePath != None:
    libPath = profilePath + '/Documents/Arduino/libraries'
    try:
        os.makedirs('{0}/{1}'.format(libPath,cname))
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
    file('{lib}/{c}/{c}.cpp'.format(lib=libPath, c=cname), 'w').write(code)
    file('{lib}/{c}/{c}.h'.format(lib=libPath, c=cname), 'w').write(header)
else:
    print 'Unable to locate arduino library directory'
