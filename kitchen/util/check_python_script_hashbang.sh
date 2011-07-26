#!/bin/sh

#
#   use like this:
#
#   find . -name '*.py' -exec util/check_python_script_hashbang.sh {} \;
#

SCRIPT=$1

FIRST=$(head -c 21 $1)
if [ ! "$FIRST" = "#!/usr/bin/env python" ] ; then
    echo "$1: FAIL, MISSING #!/usr/bin/env python"
    return 1
fi
if [ ! -x $1 ] ; then
    echo "$1: FAIL not executable"
    return 1
fi
echo "$1: okay."
return 0