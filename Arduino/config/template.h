/* Do not modify - auto-generated header file
 * Produced by {{scriptname}}

 * DESCRIPTION
 * {{ data.description|wordwrap(width=75, wrapstring='\n * ') }}
 */

#ifndef _{{data.classname|upper}}_H
#define _{{data.classname|upper}}_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

{% for def in data.defines %}
#define {{data.classname|upper}}_{{def}}
{% endfor %}

class {{data.classname}} {
private:
    {% for v in data.variables if not v.public %}
    {{'static ' if v.static }}{{v.type}} {{v.name}};
    {% endfor %}

    {% for f in data.private_functions %}
    {{f}};
    {% endfor %}

public:
    {% for f in data.public_functions %}
    {{f}};
    {% endfor %}

{% for attr in data.attribs %}
    {% if 'R' in attr.access %}
    {{attr.type}} get{{attr.name}}(int &err);
    {% endif %}
    {% if 'W' in attr.access %}
    int set{{attr.name}}({{attr.type}} {{attr.name|lower}});
    {% endif %}
{% endfor %}
};

#endif
