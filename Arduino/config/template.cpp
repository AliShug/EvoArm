/* Do not modify - auto-generated code file
 * Produced by {{scriptname}}

 * DESCRIPTION
 * {{ data.description|wordwrap(width=75, wrapstring='\n * ') }}
 */

#include "{{data.classname}}.h"

#define TemplateSerial {{data.serial}}

{% for v in data.variables if v.static %}
{{v.type}} {{data.classname}}::{{v.name}};
{% endfor %}

//// INCLUDED FROM {{data.codefile}}
{% include data.codefile %}
//// END INCLUDE

{% for attr in data.attribs %}
{% if 'R' in attr.access %}
{{attr.type}} {{data.classname}}::get{{attr.name}}(int &err) {
    unsigned char b[{{attr.size}}];
    err = read({{attr.adr}}, {{attr.size}}, b);
    if (err == {{data.classname|upper}}_ERR_OK) {
        int raw = b[0];
        {% if attr.size == 2 %}
        raw |= b[1] << 8;
        {% endif %}
        {% if attr.unit is defined %}
        return convertTo{{attr.unit}}(raw);
        {% else %}
        return raw;
        {% endif %}
    }
    else {
        return ({{attr.type}}) NULL;
    }
}

{% endif %}
{% if 'W' in attr.access %}
int {{data.classname}}::set{{attr.name}}({{attr.type}} {{attr.name|lower}}) {
    unsigned char b[{{attr.size}}];
    {% if attr.unit %}
    int raw = convertFrom{{attr.unit}}({{attr.name|lower}});
    {% else %}
    int raw = {{attr.name|lower}};
    {% endif %}
    {% if attr.min is defined %}
    if (raw < {{attr.min}}) {
        raw = {{attr.min}};
    }
    {% endif %}
    {% if attr.max is defined %}
    if (raw > {{attr.max}}) {
        raw = {{attr.max}};
    }
    {% endif %}
    b[0] = raw;
    {% if attr.size == 2 %}
    b[1] = raw >> 8;
    {% endif %}
    int err = write({{attr.adr}}, {{attr.size}}, b);
    {% if attr.onchange is defined %}
    if (err == {{data.classname|upper}}_ERR_OK) {
        {{attr.onchange|indent(8)}}
    }
    {% endif %}
    return err;
}

{% endif %}
{% endfor %}

#undef TemplateSerial
