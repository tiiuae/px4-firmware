#!/bin/bash

source /opt/ros/humble/setup.sh

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING ${arg}"

        # extract the middle part of the name between the "_"'s
        NAME=${arg}
        NAME=${NAME%_*}
        NAME=${NAME##*_}

        # for our own HW, use HW specific siging. For pixhawks, and icicle board,
        # use the PX4 default signing script and keys
        if [[ $NAME = saluki* ]]
        then
            if [[ "$SIGNING_KEY" == *hsm* ]]; then
               echo "Using HSM for signing"
               export SIGNING_TOOL=Tools/saluki-sec-scripts/sign_by_hsm.py
            else
               echo "Using keyfile script for signing"
               export SIGNING_TOOL=Tools/saluki-sec-scripts/sign_by_keyfile.py

               if [ -z "$SIGNING_ARGS" ]; then
                   if [[ "$NAME" == "saluki-nxp93" || "$NAME" == "saluki-micro" || "$NAME" == "saluki-ft" ]]; then
                       export SIGNING_ARGS=Tools/saluki-sec-scripts/test_keys/$NAME/secp384r1_test_key1.pem
                   else
                       export SIGNING_ARGS=Tools/saluki-sec-scripts/test_keys/$NAME/ed25519_test_key.pem
                   fi
               fi
            fi
        elif [ -f "boards/${arg%%_*}/${NAME}/src/toc.c" ]; then
            export SIGNING_TOOL=Tools/cryptotools.py
            unset SIGNING_ARGS
        else
            # Setting SIGNING_TOOL makes CMake build a table of contents from the board's toc.c.
            unset SIGNING_TOOL SIGNING_ARGS
        fi

        # Remove old build output, and what a configure leaves inside NuttX,
        # which a board of another chip family would otherwise build against
        rm -Rf build/${arg}
        git -C platforms/nuttx/NuttX/nuttx clean -Xfdq
        git -C platforms/nuttx/NuttX/apps clean -Xfdq
        # Build
        make -j$((`nproc`+1)) ${arg} || exit 1

        if [ -n "$SIGNING_ARGS" ]; then
            echo "Signing key: $SIGNING_ARGS"
        fi
    done
fi
