#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/vgampa/mpc_quad/src/CrazyS/rqt_rotors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/vgampa/mpc_quad/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/vgampa/mpc_quad/install/lib/python3/dist-packages:/home/vgampa/mpc_quad/build/rqt_rotors/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/vgampa/mpc_quad/build/rqt_rotors" \
    "/usr/bin/python3" \
    "/home/vgampa/mpc_quad/src/CrazyS/rqt_rotors/setup.py" \
     \
    build --build-base "/home/vgampa/mpc_quad/build/rqt_rotors" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/vgampa/mpc_quad/install" --install-scripts="/home/vgampa/mpc_quad/install/bin"
